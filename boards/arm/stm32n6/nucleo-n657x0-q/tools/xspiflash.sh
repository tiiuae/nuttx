#!/usr/bin/env bash
###############################################################################
# boards/arm/stm32n6/nucleo-n657x0-q/tools/xspiflash.sh
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
# Program a CONFIG_NUCLEO_N657X0_Q_BOOT_XSPI image into the external Octo-SPI
# NOR flash of the NUCLEO-N657X0-Q, using STM32_Programmer_CLI and the
# MX25UM51245G external loader.
#
# This is the counterpart of tools/sramload.sh:
#
#   sramload.sh   downloads an SRAM image and runs it immediately; nothing
#                 survives a reset.
#   xspiflash.sh  writes a packaged image (tools/mkimage.sh) to the external
#                 flash so the board boots standalone.
#
# The board has to be in DEV boot mode while programming: the ST-LINK needs an
# open debug port to download the external loader into SRAM and drive XSPI2
# with it.  Once programmed, switch the boot pins to boot-from-flash and reset.
#
###############################################################################

set -e

progname=$(basename "$0")

# Defaults ####################################################################

IMAGE=nuttx-signed.bin
RAWBIN=nuttx.bin
FLASHBASE=0x70000000
EXTLOADER=MX25UM51245G_STM32N6570-NUCLEO.stldr
PORT=SWD
CONNMODE=HOTPLUG
VERIFY=1
DUMP=0

usage()
{
  cat <<EOF
Usage: $progname [OPTIONS]

Program a packaged NuttX image into the NUCLEO-N657X0-Q external flash.
Put the board's boot switches in DEV boot mode first.

Options:
  -i, --input FILE     Packaged image to program     (default: $IMAGE)
  -a, --address ADDR   Flash base address            (default: $FLASHBASE)
      --el FILE        External loader: a name inside
                       \$STM32_PRG_PATH/ExternalLoader, or a full path
                       (default: $EXTLOADER)
      --port PORT      ST-LINK port                  (default: $PORT)
      --mode MODE      Connection mode: HOTPLUG/UR/NORMAL (default: $CONNMODE)
      --no-verify      Skip the read-back verification
  -d, --dump           Dump the image header before programming
  -h, --help           Show this help

Environment:
  STM32_PRG_PATH       Directory holding STM32_Programmer_CLI.  Typically
                       .../STM32CubeProgrammer/bin  (required)
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
    -i|--input)      IMAGE=$2; shift 2 ;;
    -a|--address)    FLASHBASE=$2; shift 2 ;;
    --el)            EXTLOADER=$2; shift 2 ;;
    --port)          PORT=$2; shift 2 ;;
    --mode)          CONNMODE=$2; shift 2 ;;
    --no-verify)     VERIFY=0; shift ;;
    -d|--dump)       DUMP=1; shift ;;
    -h|--help)       usage; exit 0 ;;
    *)               usage 1>&2; die "unknown option '$1'" ;;
  esac
done

# Locate the tools ############################################################

if [ -z "$STM32_PRG_PATH" ]; then
  die "STM32_PRG_PATH is not set.  Point it at the STM32CubeProgrammer 'bin'
       directory, e.g.
         export STM32_PRG_PATH=\$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
fi

PROGRAMMER="$STM32_PRG_PATH/STM32_Programmer_CLI"
SIGNTOOL="$STM32_PRG_PATH/STM32_SigningTool_CLI"

[ -x "$PROGRAMMER" ] ||
  die "'$PROGRAMMER' not found or not executable (check STM32_PRG_PATH)"

# The external loader may be given as a bare name or as a path.

case "$EXTLOADER" in
  */*) ;;
  *)   EXTLOADER="$STM32_PRG_PATH/ExternalLoader/$EXTLOADER" ;;
esac

if [ ! -f "$EXTLOADER" ]; then
  echo "$progname: ERROR: external loader '$EXTLOADER' not found." 1>&2
  echo "$progname: STM32N6 loaders available here:" 1>&2
  ls "$STM32_PRG_PATH/ExternalLoader" 2>/dev/null |
    grep -i "N6" | sed 's/^/    /' 1>&2
  exit 1
fi

# Sanity-check the image ######################################################

[ -f "$IMAGE" ] ||
  die "'$IMAGE' not found.  Build a CONFIG_NUCLEO_N657X0_Q_BOOT_XSPI
       configuration, or package a raw binary with tools/mkimage.sh."

# A packaged image starts with the STM32 header magic, 0x53544d32 ("STM2").
# Programming a raw nuttx.bin instead is an easy mistake and the boot ROM
# would simply reject it, so catch it here.

magic=$(head -c 4 "$IMAGE" | od -An -c | tr -d ' \n')
if [ "$magic" != "STM2" ]; then
  die "'$IMAGE' does not start with the STM32 header magic (\"STM2\").
       It looks like a raw binary; run tools/mkimage.sh on it first."
fi

# Warn if the packaged image is older than the raw binary it came from.

if [ -f "$RAWBIN" ] && [ "$RAWBIN" -nt "$IMAGE" ]; then
  echo "$progname: WARNING: '$RAWBIN' is newer than '$IMAGE';" \
       "re-run tools/mkimage.sh?" 1>&2
fi

# The dump is a diagnostic aid only: never let it abort the programming.

if [ "$DUMP" -eq 1 ]; then
  if [ -x "$SIGNTOOL" ]; then
    "$SIGNTOOL" -dump "$IMAGE" ||
      echo "$progname: WARNING: header dump failed; continuing" 1>&2
  else
    echo "$progname: WARNING: '$SIGNTOOL' not found; skipping header dump" 1>&2
  fi
fi

# Program #####################################################################

echo "$progname: programming '$IMAGE' at $FLASHBASE"
echo "$progname:   loader : $(basename "$EXTLOADER")"
echo "$progname:   size   : $(wc -c < "$IMAGE") bytes"
echo "$progname: the board must be in DEV boot mode for this to work."

set -- -c "port=$PORT" "mode=$CONNMODE" \
       -el "$EXTLOADER"                 \
       -w "$IMAGE" "$FLASHBASE"

if [ "$VERIFY" -eq 1 ]; then
  set -- "$@" -v
fi

"$PROGRAMMER" "$@"

cat <<EOF
$progname: done.  To run it:
    1. set the boot switches to boot-from-flash
    2. press NRST
    3. the console appears on USART1 via the ST-LINK VCP

Note that the debug port is not available in boot-from-flash mode, so
switch back to DEV boot mode before using sramload.sh or the debugger
again.  See CONFIG_NUCLEO_N657X0_Q_XIPBOOT_TRACE for diagnosing a boot
that does not reach NSH.
EOF
