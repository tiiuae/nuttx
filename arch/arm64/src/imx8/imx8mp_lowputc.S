/****************************************************************************
 * arch/arm64/src/imx8/imx8mp_lowputs.S
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
 ****************************************************************************
 *
 *    DESCRIPTION
 *       Wrapper for early printk
 *
 ***************************************************************************/

#include <nuttx/config.h>

#include "arm64_macro.inc"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "imx8mp_lowputc.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* i.MX8 M Plus UART1 base */

#define UART1_BASE_ADDRESS 0x30860000

/* i.MX8 M Plus registers:
 * UTXD = Transmitter register / write 1 octet to TX FIFO
 * USR1 = Status register
 * USR1_TRDY = TX FIFO below threshold level, default level is <= 2 octets
 */

#define UTXD               (0x40)
#define USR1               (0x94)
#define USR1_TRDY          (1 << 13)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

GTEXT(arm64_earlyprintinit)
SECTION_FUNC(text, arm64_earlyprintinit)
    /* REVISIT: Set baudrate etc. now assumes that u-boot has set us up */
    ret

/* i.MX8MP wait UART to be ready to transmit
 * xb: register which contains the UART base address
 * rc: scratch register
 */
.macro early_uart_ready xb, rc
1:
    ldr   \rc, [\xb, #USR1]  /* <- Flag register */
    tst   \rc, #USR1_TRDY    /* Check FIFO READY bit */
    beq   1b                 /* Wait for the UART to be ready */
.endm

/* i.MX8QM LPUART transmit character
 * xb: register which contains the UART base address
 * wt: register which contains the character to transmit */
.macro early_uart_transmit xb, wt
    str   \wt, [\xb, #UTXD]  /* -> Data Register */
.endm

/*
 * Print a character on the UART - this function is called by C
 * x0: character to print
 */
GTEXT(arm64_lowputc)
SECTION_FUNC(text, arm64_lowputc)
    ldr   x15, =UART1_BASE_ADDRESS
    early_uart_ready x15, w1
    early_uart_transmit x15, w0
    ret
