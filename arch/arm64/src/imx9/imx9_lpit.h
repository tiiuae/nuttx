/****************************************************************************
 * arch/arm64/src/imx9/imx9_lpit.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_LPIT_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_LPIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LPIT_HANDLE_INITIALIZER {-1, 0, false}

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*lpit_callback_t)(void *arg);
typedef struct
{
  uint8_t lpit;
  uint8_t ch_mask;
  bool reserved;
} lpit_handle_t;

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

int imx9_lpit_reserve(lpit_handle_t *handle);
int imx9_lpit_release(lpit_handle_t *handle, int reserved_timer);
int imx9_lpit_release_ch(lpit_handle_t *handle, int ch);
int imx9_lpit_reserve_periodic_ch(lpit_handle_t *handle, uint32_t freq,
                                  lpit_callback_t isr, void *arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_LPIT_H */
