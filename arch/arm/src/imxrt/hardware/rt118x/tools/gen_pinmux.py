"""Generate imxrt118x_pinmux.h from the RT1189 SDK fsl_iomuxc.h."""

import re
import sys

SRC = "/home/jlaitine/TII/Documents/saluki/mcuxpresso-sdk/mcuxsdk/devices/RT/RT1180/MIMXRT1189/drivers/fsl_iomuxc.h"

BANKS = {
    0x42A10000: dict(bank_bit=0, ctl_first=0x010, pad_delta=0x248, dsy_first=0x4A0, name="main"),
    0x443C0000: dict(bank_bit=1, ctl_first=0x000, pad_delta=0x074, dsy_first=0x0E8, name="aon"),
}

pat = re.compile(
    r"#define\s+(\w+)\s+"
    r"(0x[0-9A-Fa-f]+)U,\s*"
    r"(0x[0-9A-Fa-f]+)U,\s*"
    r"(0x[0-9A-Fa-f]+|0)U?,\s*"
    r"(0x[0-9A-Fa-f]+|0)U?,\s*"
    r"(0x[0-9A-Fa-f]+)U"
)

def bank_of(addr):
    for base in BANKS:
        if (addr & ~0xFFF) == base:
            return base
    raise RuntimeError(f"Unknown bank for 0x{addr:x}")

entries = []
data = open(SRC).read()
for m in pat.finditer(data):
    name    = m.group(1)
    ctl     = int(m.group(2), 16)
    mux     = int(m.group(3), 16)
    dsy_reg = int(m.group(4), 16) if m.group(4) != "0" else 0
    dsy_val = int(m.group(5), 16) if m.group(5) != "0" else 0
    pad_reg = int(m.group(6), 16)

    pad_bank_base = bank_of(ctl)
    b = BANKS[pad_bank_base]
    assert pad_reg == ctl + b["pad_delta"], (
        f"{name}: PAD offset mismatch: pad=0x{pad_reg:x} ctl=0x{ctl:x} bank={b['name']}"
    )
    pad_idx = (ctl - pad_bank_base - b["ctl_first"]) // 4
    assert 0 <= pad_idx < 256, f"{name}: pad_idx out of range ({pad_idx})"

    if dsy_reg:
        dsy_bank_base = bank_of(dsy_reg)
        db = BANKS[dsy_bank_base]
        dsy_idx = (dsy_reg - dsy_bank_base - db["dsy_first"]) // 4
        assert 0 <= dsy_idx < 511, f"{name}: dsy_idx out of range ({dsy_idx})"
        dsy_bank_bit = db["bank_bit"]
    else:
        dsy_idx = None
        dsy_bank_bit = 0

    entries.append((name, b["bank_bit"], pad_idx, mux, dsy_bank_bit, dsy_idx, dsy_val))

print(f"# {len(entries)} pin/alt combos parsed", file=sys.stderr)

OUT = "/home/jlaitine/TII/px4-firmware-public/platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/hardware/rt118x/imxrt118x_pinmux.h"

hdr = '''/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_pinmux.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_PINMUX_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rt118x/imxrt118x_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Each IOMUXC_PAD_<pin>_<alt> constant is a packed uint32_t suitable for
 * use as the padcfg argument of IOMUX_PIN(padcfg, pad_ctl, sion) or
 * IOMUX_GPIO(padcfg, pad_ctl, gpio).  The layout and packing is defined
 * in imxrt118x_iomuxc.h.
 */
'''

lines = [hdr]
entries.sort(key=lambda e: (e[1], e[2], e[3]))
prev_key = None
for name, pbank, pad_idx, mux, dbank, dsy_idx, dsy_val in entries:
    key = (pbank, pad_idx)
    if prev_key is not None and prev_key != key:
        lines.append("")
    prev_key = key

    macro = f"IOMUXC_PAD_{name.removeprefix('IOMUXC_')}"
    if dsy_idx is None:
        args = f"({pbank}, {pad_idx:>3}, 0x{mux:x}, 0, IMXRT_PADCFG_NO_DSYIDX, 0)"
    else:
        args = f"({pbank}, {pad_idx:>3}, 0x{mux:x}, {dbank}, {dsy_idx:>3}, 0x{dsy_val:x})"
    lines.append(f"#define {macro:<70s} IMXRT_PADCFG{args}")

lines.append("")
lines.append("#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_PINMUX_H */")

open(OUT, "w").write("\n".join(lines))
print(f"Wrote {OUT} ({len(entries)} entries)", file=sys.stderr)
