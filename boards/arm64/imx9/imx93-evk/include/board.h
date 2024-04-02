/****************************************************************************
 * boards/arm64/imx9/imx93-evk/include/board.h
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

#ifndef __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default PAD configurations */

#define IOMUX_LPI2C_DEFAULT  (IOMUXC_PAD_OD_ENABLE | IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6)
#define IOMUX_LPSPI_DEFAULT  (IOMUXC_PAD_PU_ON     | IOMUXC_PAD_FSEL_FAST  | IOMUXC_PAD_DSE_X6)
#define IOMUX_GPIO_DEFAULT   (IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X6)

/* LPI2Cs */

#define MUX_LPI2C1_SCL       IOMUX_CFG(IOMUXC_PAD_I2C1_SCL_LPI2C1_SCL, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPI2C1_SDA       IOMUX_CFG(IOMUXC_PAD_I2C1_SDA_LPI2C1_SDA, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)

/* I2C reset functionality */

#define GPIO_LPI2C1_SCL_RESET  (GPIO_PORT1 | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define GPIO_LPI2C1_SDA_RESET  (GPIO_PORT1 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/* LPSPIs */

#define MUX_LPSPI3_SCK       IOMUX_CFG(IOMUXC_PAD_GPIO_IO11_LPSPI3_SCK,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI3_MISO      IOMUX_CFG(IOMUXC_PAD_GPIO_IO10_LPSPI3_SOUT, IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI3_MOSI      IOMUX_CFG(IOMUXC_PAD_GPIO_IO09_LPSPI3_SIN,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI6_SCK       IOMUX_CFG(IOMUXC_PAD_GPIO_IO03_LPSPI6_SCK,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI6_MISO      IOMUX_CFG(IOMUXC_PAD_GPIO_IO02_LPSPI6_SOUT, IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI6_MOSI      IOMUX_CFG(IOMUXC_PAD_GPIO_IO01_LPSPI6_SIN,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)

/* SPI CS */

#define MUX_LPSPI3_CS        IOMUX_CFG(IOMUXC_PAD_GPIO_IO08_GPIO2_IO08,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_LPSPI3_CS       (GPIO_PORT2 | GPIO_PIN8 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define MUX_LPSPI6_CS        IOMUX_CFG(IOMUXC_PAD_GPIO_IO00_GPIO2_IO00,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_LPSPI6_CS       (GPIO_PORT2 | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H */
