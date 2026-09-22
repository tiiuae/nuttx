#!/usr/bin/env bash
###############################################################################
# boards/arm/stm32n6/nucleo-n657x0-q/tools/mkimage.sh
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
###############################################################################
#
# Wrap nuttx.bin in an STM32 v2.3 boot header so that the STM32N6 boot ROM
# accepts it from the external Octo-SPI NOR flash on XSPI2.
#
# The header declares a load address of 0xffffffff, which tells the ROM
# "execute in place, do not relocate".  The ROM therefore leaves XSPI2 in
# memory-mapped mode and branches straight to the entry point, which is the
# _flash_start stub in src/stm32_xipboot.S.  That stub is what copies the
# image from flash into SRAM (see scripts/flash.ld).
#
# The "-align" option makes the signing tool pad the header out to 0x400
# bytes, which matches ORIGIN of the "flash" region in scripts/flash.ld.
#
###############################################################################

set -e

progname=$(basename "$0")

# Defaults ####################################################################

ELF=nuttx
BIN=nuttx.bin
OUT=
FLASHBASE=0x70000000
PAYLOAD_OFFSET=0x400
LOADADDR=0xffffffff
ENTRY=
OPTFLAGS=0x80000000
HDRVERSION=2.3
IMGTYPE=fsbl
CROSSDEV=${CROSSDEV:-arm-none-eabi-}
VERBOSE=0

usage()
{
  cat <<EOF
Usage: $progname [OPTIONS]

Package nuttx.bin into a headered image that the STM32N6 boot ROM can start
from the external flash at $FLASHBASE.

Options:
  -i, --input FILE     Raw binary to package         (default: $BIN)
  -e, --elf FILE       ELF used for consistency checks and to derive the
                       entry point                   (default: $ELF)
  -o, --output FILE    Packaged image                (default: <input>-signed.bin)
      --entry ADDR     Override the header entry point
      --load ADDR      Override the header load address (default: $LOADADDR)
      --flash-base A   Base of the memory-mapped flash (default: $FLASHBASE)
      --offset OFF     Header size / payload offset   (default: $PAYLOAD_OFFSET)
  -v, --verbose        Dump the resulting header
  -h, --help           Show this help

Environment:
  STM32_PRG_PATH       Directory holding STM32_SigningTool_CLI.  Typically
                       .../STM32CubeProgrammer/bin  (required)
  CROSSDEV             Toolchain prefix used for readelf/objdump
                       (default: arm-none-eabi-)
EOF
}

die()
{
  echo "$progname: ERROR: $*" 1>&2
  exit 1
}

# Parse the command line ######################################################

while [ $# -gt 0 ]; do
  case "$1" in
    -i|--input)      BIN=$2; shift 2 ;;
    -e|--elf)        ELF=$2; shift 2 ;;
    -o|--output)     OUT=$2; shift 2 ;;
    --entry)         ENTRY=$2; shift 2 ;;
    --load)          LOADADDR=$2; shift 2 ;;
    --flash-base)    FLASHBASE=$2; shift 2 ;;
    --offset)        PAYLOAD_OFFSET=$2; shift 2 ;;
    -v|--verbose)    VERBOSE=1; shift ;;
    -h|--help)       usage; exit 0 ;;
    *)               usage 1>&2; die "unknown option '$1'" ;;
  esac
done

[ -n "$OUT" ] || OUT="${BIN%.bin}-signed.bin"

# Locate the signing tool #####################################################

if [ -z "$STM32_PRG_PATH" ]; then
  die "STM32_PRG_PATH is not set.  Point it at the STM32CubeProgrammer 'bin'
       directory, e.g.
         export STM32_PRG_PATH=\$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
fi

SIGNTOOL="$STM32_PRG_PATH/STM32_SigningTool_CLI"

[ -x "$SIGNTOOL" ] ||
  die "'$SIGNTOOL' not found or not executable (check STM32_PRG_PATH)"

[ -f "$BIN" ] || die "input binary '$BIN' not found"

# Cross-check the ELF against the linker script ###############################
#
# These checks are cheap and catch the two mistakes that produce an image
# which silently fails to boot: a payload offset that disagrees with the
# "flash" region in flash.ld, and a stale entry point.

PAYLOAD_BASE=$(printf "0x%x" $((FLASHBASE + PAYLOAD_OFFSET)))

# 0xffffffff selects XIP.  Any other load address selects LRUN: the boot ROM
# copies the payload to LOADADDR and branches to its SRAM entry point.

if [ "$LOADADDR" = "0xffffffff" ] || [ "$LOADADDR" = "0xFFFFFFFF" ]; then
  XIP=1
else
  XIP=0
fi

if [ -f "$ELF" ]; then
  xipboot_lma=$("${CROSSDEV}objdump" -h "$ELF" 2>/dev/null |
                awk '$2 == ".xipboot" { print "0x" $5 }')

  if [ -z "$ENTRY" ]; then
    ENTRY=$("${CROSSDEV}readelf" -h "$ELF" 2>/dev/null |
            awk '/Entry point address/ { print $NF }')
    if [ -z "$ENTRY" ] || [ $((ENTRY)) -eq 0 ]; then
      ENTRY=
    fi
  fi

  if [ "$XIP" -eq 1 ]; then
    [ -n "$xipboot_lma" ] ||
      echo "$progname: WARNING: no .xipboot section in '$ELF';" \
           "is this an XIP build?" 1>&2

    if [ -n "$xipboot_lma" ] &&
       [ $((xipboot_lma)) -ne $((PAYLOAD_BASE)) ]; then
      die ".xipboot is loaded at $xipboot_lma but the header puts the payload
       at $PAYLOAD_BASE.  Update ORIGIN of the 'flash' region in
       scripts/flash.ld, or pass --offset/--flash-base."
    fi

    if [ -n "$ENTRY" ] &&
       { [ $((ENTRY)) -lt $((FLASHBASE)) ] ||
         [ $((ENTRY)) -ge $((FLASHBASE + 0x4000000)) ]; }; then
      die "the ELF entry point $ENTRY is outside external flash; an XIP image
       must enter the .xipboot stub in the flash window."
    fi
  else
    if [ -n "$ENTRY" ] && [ $((ENTRY)) -ge $((FLASHBASE)) ]; then
      die "the ELF entry point $ENTRY is in external flash, but the header
       requests an SRAM load at $LOADADDR."
    fi
  fi
fi

# Fall back to the first instruction of the selected execution region.  Bit 0
# selects Thumb state, required by the Cortex-M55.

if [ -z "$ENTRY" ]; then
  if [ "$XIP" -eq 1 ]; then
    ENTRY=$(printf "0x%x" $((PAYLOAD_BASE | 1)))
  else
    ENTRY=$(printf "0x%x" $((LOADADDR | 1)))
  fi
fi

# Package #####################################################################

echo "$progname: packaging '$BIN' -> '$OUT'"
echo "$progname:   flash base   : $FLASHBASE"
echo "$progname:   payload at   : $PAYLOAD_BASE (header size $PAYLOAD_OFFSET)"
echo "$progname:   load address : $LOADADDR"
echo "$progname:   entry point  : $ENTRY"

rm -f "$OUT"

"$SIGNTOOL" -bin "$BIN"           \
            -nk                   \
            -t "$IMGTYPE"         \
            -hv "$HDRVERSION"     \
            -of "$OPTFLAGS"       \
            -la "$LOADADDR"       \
            -ep "$ENTRY"          \
            -align                \
            -o "$OUT"             \
            -s

[ -f "$OUT" ] || die "$SIGNTOOL did not produce '$OUT'"

# The signing tool is not always reliable about its exit status, and it pads
# the tail of the payload, so the header size cannot simply be derived from
# the file sizes.  Verify the payload content instead: the first bytes of the
# raw binary must appear at PAYLOAD_OFFSET in the packaged image.

ncmp=64
binsize=$(wc -c < "$BIN")
[ "$binsize" -ge "$ncmp" ] || ncmp=$binsize

if ! cmp -s -n "$ncmp" -i "$((PAYLOAD_OFFSET)):0" "$OUT" "$BIN"; then
  die "'$BIN' did not land at offset $PAYLOAD_OFFSET of '$OUT'.
       The signing tool changed its header layout; adjust --offset and the
       'flash' region in scripts/flash.ld to match."
fi

chmod u+w "$OUT"

if [ "$VERBOSE" -eq 1 ]; then
  "$SIGNTOOL" -dump "$OUT"
fi

echo "$progname: done."
echo "$progname: Set the boot switches to DEV boot mode, then program with:"
echo "    \$STM32_PRG_PATH/STM32_Programmer_CLI -c port=SWD mode=HOTPLUG \\"
echo "        -el \$STM32_PRG_PATH/ExternalLoader/MX25UM51245G_STM32N6570-NUCLEO.stldr \\"
echo "        -w $OUT $FLASHBASE"
echo "$progname: Then switch the boot pins to boot-from-flash and reset."
