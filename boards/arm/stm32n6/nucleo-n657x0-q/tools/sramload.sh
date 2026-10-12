#!/usr/bin/env bash
###############################################################################
# boards/arm/stm32n6/nucleo-n657x0-q/tools/sramload.sh
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
# Download a CONFIG_NUCLEO_N657X0_Q_BOOT_SRAM image into the board's AXISRAM
# and start it, using STM32_Programmer_CLI over the on-board ST-LINK.
#
# The STM32N657X0 has no internal flash.  In DEV boot mode the boot ROM parks
# the CPU with the debug port open and never sets up a vector table, so the
# debugger has to do three things:
#
#   1. Write the image to CONFIG_RAM_START (0x34000400).  The image begins
#      with the vector table, because scripts/sram.ld puts *(.vectors) first
#      in .text.
#   2. Set MSP to vector 0 and PC to the image entry point (__start).
#   3. Release the core.
#
# Nothing persists across a power cycle; re-run this after every reset.  Use
# the CONFIG_NUCLEO_N657X0_Q_BOOT_XSPI configuration and tools/mkimage.sh if
# you want the board to boot standalone from the external flash.
#
###############################################################################

set -e

progname=$(basename "$0")

# Defaults ####################################################################

ELF=nuttx
BIN=nuttx.bin
RAMSTART=0x34000400
PORT=SWD
CONNMODE=HOTPLUG
ENTRY=
MSP=
RUN=1
CROSSDEV=${CROSSDEV:-arm-none-eabi-}

usage()
{
  cat <<EOF
Usage: $progname [OPTIONS]

Download an SRAM (DEV boot mode) NuttX image to the NUCLEO-N657X0-Q and run
it.  Put the board's boot switches in DEV boot mode first.

Options:
  -i, --input FILE     Raw binary to download      (default: $BIN)
  -e, --elf FILE       ELF providing the entry point (default: $ELF)
  -a, --address ADDR   SRAM load address           (default: $RAMSTART)
      --entry ADDR     Override the entry point (PC)
      --msp ADDR       Override the initial stack pointer
      --port PORT      ST-LINK port                (default: $PORT)
      --mode MODE      Connection mode: HOTPLUG/UR/NORMAL (default: $CONNMODE)
  -n, --no-run         Leave the core halted at the entry point, ready for
                       a debugger to attach
  -h, --help           Show this help

Environment:
  STM32_PRG_PATH       Directory holding STM32_Programmer_CLI.  Typically
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
    -a|--address)    RAMSTART=$2; shift 2 ;;
    --entry)         ENTRY=$2; shift 2 ;;
    --msp)           MSP=$2; shift 2 ;;
    --port)          PORT=$2; shift 2 ;;
    --mode)          CONNMODE=$2; shift 2 ;;
    -n|--no-run)     RUN=0; shift ;;
    -h|--help)       usage; exit 0 ;;
    *)               usage 1>&2; die "unknown option '$1'" ;;
  esac
done

# Locate the programmer #######################################################

if [ -z "$STM32_PRG_PATH" ]; then
  die "STM32_PRG_PATH is not set.  Point it at the STM32CubeProgrammer 'bin'
       directory, e.g.
         export STM32_PRG_PATH=\$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
fi

PROGRAMMER="$STM32_PRG_PATH/STM32_Programmer_CLI"

[ -x "$PROGRAMMER" ] ||
  die "'$PROGRAMMER' not found or not executable (check STM32_PRG_PATH)"

[ -f "$BIN" ] || die "input binary '$BIN' not found"

# Work out where to start #####################################################

if [ -f "$ELF" ]; then

  # Refuse an external-flash image: its LMAs are in XSPI2, so downloading it
  # to SRAM would put the wrong bytes at the wrong addresses.

  if "${CROSSDEV}objdump" -h "$ELF" 2>/dev/null | grep -q '\.xipboot'; then
    die "'$ELF' is a CONFIG_NUCLEO_N657X0_Q_BOOT_XSPI build (it has a
       .xipboot section).  Package it with tools/mkimage.sh and program it
       to the external flash instead."
  fi

  text_vma=$("${CROSSDEV}objdump" -h "$ELF" 2>/dev/null |
             awk '$2 == ".text" { print "0x" $4 }')

  if [ -n "$text_vma" ] && [ $((text_vma)) -ne $((RAMSTART)) ]; then
    die "'$ELF' links .text at $text_vma but the download address is
       $RAMSTART.  Pass -a $text_vma, or check CONFIG_RAM_START."
  fi

  if [ -z "$ENTRY" ]; then
    ENTRY=$("${CROSSDEV}readelf" -h "$ELF" 2>/dev/null |
            awk '/Entry point address/ { print $NF }')
  fi
fi

[ -n "$ENTRY" ] || die "cannot determine the entry point; pass --entry"
[ $((ENTRY)) -ne 0 ] || die "entry point is 0; is '$ELF' a linked executable?"

# The initial stack pointer is vector 0, i.e. the very first word of the
# image.  (arm_vectors.c puts IDLE_STACK there.)

if [ -z "$MSP" ]; then
  MSP=0x$(od -An -tx4 -N4 --endian=little "$BIN" | tr -d ' \n')
fi

[ $((MSP)) -ne 0 ] || die "initial MSP read from '$BIN' is 0; pass --msp"

# Download and go #############################################################

echo "$progname: downloading '$BIN' to $RAMSTART"
echo "$progname:   MSP : $MSP"
echo "$progname:   PC  : $ENTRY"

set -- -c "port=$PORT" "mode=$CONNMODE" \
       -halt                            \
       -w "$BIN" "$RAMSTART"            \
       -coreReg "MSP=$MSP" "PC=$ENTRY"

if [ "$RUN" -eq 1 ]; then
  set -- "$@" -run
fi

"$PROGRAMMER" "$@"

if [ "$RUN" -eq 1 ]; then
  echo "$progname: running.  Console is on USART1 via the ST-LINK VCP."
else
  echo "$progname: core halted at $ENTRY; attach a debugger to continue."
fi
