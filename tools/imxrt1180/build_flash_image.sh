#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# build_flash_image.sh - assemble the MIMXRT1180-EVK bootable FlexSPI NOR
#                        image (flash.bin) from the freshly built
#                        Cortex-M33 NuttX image, using NXP SPSDK to
#                        generate the AHAB container.
#
# Only the Cortex-M33 image is bootable: it carries the FCB and the AHAB
# container that the ROM consumes.  The Cortex-M7 image is a raw XIP
# payload (nuttx.bin) flashed separately at offset 0x80000; it needs no
# packaging and is therefore not handled here.
#
# Modeled after saluki_bootloader_v2/tools/imx9/create_nxp_image.sh:
# tools are downloaded / installed on first use into a private cache
# directory, so a plain "make" bootstraps everything without the user
# having to install SPSDK or fetch NXP firmware manually.
#
# Usage examples:
#
#   tools/imxrt1180/build_flash_image.sh --m33 nuttx.bin --out flash.bin
#
# NXP EULA note:
#
#   * The SPSDK Python package is BSD-3-Clause.
#   * The ELE FW AHAB container is proprietary NXP object code covered by
#     the LA_OPT NXP Software License (v56 April 2024).  We fetch it from
#     NXP's public mcux-sdk mirror at a pinned commit and never commit it
#     to this repo.  Running the script implies acceptance of the EULA
#     (same convention as iMX9 --auto-accept).
#
# ---------------------------------------------------------------------------

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Pinned versions.  Bumping these is a code review.
SPSDK_VERSION="3.11.0"

# Flash layout constants (must match flash-m33.ld).
M33_LOAD_ADDR="0x2800B000"
FLASH_ORIGIN="0x28000000"

# The M33 image is confined to the first 512 KB of NOR; the Cortex-M7
# image starts right after it (see boards/.../scripts/flash.ld).
M33_MAX_SIZE=$((512 * 1024))

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

M33_BIN=""
OUT_BIN="flash.bin"

usage() {
    sed -n '2,32p' "$0"
    exit 0
}

err() { printf "error: %s\n" "$*" >&2; exit 1; }

while [ $# -gt 0 ]; do
    case "$1" in
        --m33)   M33_BIN="$2";  shift 2 ;;
        --out)   OUT_BIN="$2";  shift 2 ;;
        -h|--help) usage ;;
        *) err "unknown argument: $1" ;;
    esac
done

# Only one mode: --m33 <nuttx.bin> produces FCB @ 0x400 + M33 AHAB @
# 0x1000.
#
# ELE firmware is NOT injected into the AHAB stack here: the RT118x ROM
# does not auto-load ELE FW from AHAB.  The driver embeds the FW via
# .incbin (see arch/arm/src/imxrt/imxrt118x_ele_fw.c) and hands it to
# the ELE at runtime via LOAD_FW.

[ -n "${M33_BIN}" ] || err "--m33 <nuttx.bin> is required"
[ -f "${M33_BIN}" ] || err "M33 nuttx binary '${M33_BIN}' not found"

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

TOP_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
CACHE_DIR="${TOP_DIR}/tools/imxrt1180/.cache"
VENV_DIR="${CACHE_DIR}/spsdk-venv"
WORK_DIR="${CACHE_DIR}/build-$$"
NXPIMAGE="${VENV_DIR}/bin/nxpimage"

mkdir -p "${CACHE_DIR}"

cleanup() { rm -rf "${WORK_DIR}"; }
trap cleanup EXIT

# ---------------------------------------------------------------------------
# 1. Bootstrap SPSDK into a private venv (idempotent)
# ---------------------------------------------------------------------------

if [ ! -x "${NXPIMAGE}" ] || \
   ! "${NXPIMAGE}" --version 2>/dev/null | grep -q "version ${SPSDK_VERSION}"
then
    echo "Installing NXP SPSDK ${SPSDK_VERSION} into ${VENV_DIR}"
    rm -rf "${VENV_DIR}"
    python3 -m venv "${VENV_DIR}"
    "${VENV_DIR}/bin/pip" install --quiet --upgrade pip
    "${VENV_DIR}/bin/pip" install --quiet "spsdk==${SPSDK_VERSION}"
fi

# ---------------------------------------------------------------------------
# 2. (nothing to do here — ELE FW is embedded via the driver's .incbin,
#    fetched at build time by arch/arm/src/imxrt/Make.defs)
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# 3. Extract the FCB + XIP code from the input image
#
# The M33 linker script places the FCB at VMA 0x28000400 and the XIP code at VMA 0x2800B000.
# arm-none-eabi-objcopy lays these out as a flat binary starting from the
# lowest section VMA, so inside the input binary the FCB sits at file
# offset 0 and the code sits at 0xB000 - 0x400 = 0xAC00.
# ---------------------------------------------------------------------------

mkdir -p "${WORK_DIR}"

FCB_BIN="${WORK_DIR}/fcb.bin"
CODE_BIN="${WORK_DIR}/code.bin"

SRC_BIN="${M33_BIN}"
LOAD_ADDR="${M33_LOAD_ADDR}"

SRC_SIZE="$(stat -c%s "${SRC_BIN}")"

# FCB: 512 B at file offset 0
dd if="${SRC_BIN}" of="${FCB_BIN}" bs=1 count=512 status=none

# XIP code: everything from file offset 0xAC00 (= 0xB000 - 0x400) onward.
CODE_OFF=$((0xB000 - 0x400))
CODE_SIZE=$((SRC_SIZE - CODE_OFF))
[ "${CODE_SIZE}" -gt 0 ] || \
    err "input binary shorter than expected (code offset 0x${CODE_OFF})"
dd if="${SRC_BIN}" of="${CODE_BIN}" bs=1 skip=${CODE_OFF} count=${CODE_SIZE} \
   status=none

# ---------------------------------------------------------------------------
# 4. Emit the AHAB config YAML for the application container
#
# The container has exactly one image entry (the M33 NuttX binary).  We wrap it in an outer AHAB image list that also
# contains the pre-built NXP ELE FW binary container (as a
# binary_container entry) so nxpimage stitches both into a single
# flash-ready blob.
#
# Because the code is linked for FlexSPI XIP at 0x2800B000, the code
# must physically land at flash offset 0xB000 = load_address -
# 0x28000000.  nxpimage's "image_offset" is measured from the start of
# the AHAB image (not the container header), so:
#
#   flash origin           : 0x28000000
#   AHAB blob starts       : 0x00001000  (see step 5)
#   code flash target      : 0xB000
#   image_offset in AHAB   : 0xB000 - 0x1000 = 0xA000
# ---------------------------------------------------------------------------

AHAB_FLASH_OFFSET=0x1000
CODE_FLASH_OFFSET=$(( LOAD_ADDR - FLASH_ORIGIN ))
IMAGE_OFFSET=$(( CODE_FLASH_OFFSET - AHAB_FLASH_OFFSET ))

if [ "${IMAGE_OFFSET}" -le 0 ]; then
    err "computed image_offset (${IMAGE_OFFSET}) is not positive - \
the XIP address must be greater than the AHAB blob's flash offset."
fi

AHAB_YAML="${WORK_DIR}/ahab.yaml"
AHAB_BIN="${WORK_DIR}/ahab.bin"

cat > "${AHAB_YAML}" <<EOF
# Auto-generated by tools/imxrt1180/build_flash_image.sh.
#
# The M33 (or M33-stub) app is packaged as a single AHAB container.  The
# NXP EdgeLock Enclave firmware is NOT included here — the RT118x boot
# ROM does not auto-load it from AHAB; instead the driver embeds the FW
# via .incbin (see arch/arm/src/imxrt/imxrt118x_ele_fw.c) and hands it
# to the ELE at runtime via LOAD_FW.
family: mimxrt1189
revision: latest
target_memory: standard
output: ${AHAB_BIN}
output_format: bin
containers:
  - container:
      srk_set: none
      fuse_version: 0
      sw_version: 0
      images:
        - image_path: ${CODE_BIN}
          image_offset: '$(printf '0x%X' "${IMAGE_OFFSET}")'
          load_address: '${LOAD_ADDR}'
          entry_point: '${LOAD_ADDR}'
          image_type: executable
          core_id: cortex-m33
          is_encrypted: false
          hash_type: sha256
EOF

echo "Building AHAB image (nxpimage)"
"${NXPIMAGE}" ahab export -c "${AHAB_YAML}"

# ---------------------------------------------------------------------------
# 5. Assemble flash.bin
#
#   flash.bin = zero pad ... (FCB @ 0x400) ... AHAB image @ 0x1000 ...
#
# nxpimage's AHAB image is already laid out with the internal
# padding-to-1KB alignment between containers per RM Table 80.
# ---------------------------------------------------------------------------

AHAB_SIZE="$(stat -c%s "${AHAB_BIN}")"

TOTAL=$((0x1000 + AHAB_SIZE))

# The M7 image lives at flash offset 0x80000; refuse to build an M33 image
# that would run into it.
if [ "${TOTAL}" -gt "${M33_MAX_SIZE}" ]; then
    err "M33 image (${TOTAL} B) exceeds the ${M33_MAX_SIZE} B reserve; \
it would overlap the Cortex-M7 image at flash offset 0x80000."
fi

rm -f "${OUT_BIN}"
dd if=/dev/zero of="${OUT_BIN}" bs=1 count="${TOTAL}" status=none
dd if="${FCB_BIN}"  of="${OUT_BIN}" bs=1 seek=1024 conv=notrunc status=none
dd if="${AHAB_BIN}" of="${OUT_BIN}" bs=1 seek=4096 conv=notrunc status=none

echo "Wrote ${OUT_BIN} (FCB@0x400, M33 AHAB@0x1000, ${TOTAL} B)"
