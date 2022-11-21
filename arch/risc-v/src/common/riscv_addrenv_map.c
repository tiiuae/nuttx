/****************************************************************************
 * arch/risc-v/src/common/riscv_addrenv_map.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pgalloc.h"

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_pa_to_va
 *
 * Description:
 *   Map phy address to virtual address.  Not supported by all architectures.
 *
 *   REVISIT:  Should this not then be conditional on having that
 *   architecture-specific support?
 *
 * Input Parameters:
 *   pa - The phy address to be mapped.
 *
 * Returned Value:
 *   Virtual address on success; NULL on failure.
 *
 ****************************************************************************/

FAR void *up_addrenv_pa_to_va(uintptr_t pa)
{
  /* This can only translate page pool addresses for now */

  return (FAR void *)riscv_pgvaddr(pa);
}

/****************************************************************************
 * Name: up_addrenv_va_to_pa
 *
 * Description:
 *   Map virtual address to phy address.  Not supported by all architectures.
 *
 *   REVISIT:  Should this not then be conditional on having that
 *   architecture-specific support?
 *
 * Input Parameters:
 *   va - The virtual address to be mapped.  Not supported by all
 *        architectures.
 *
 * Returned Value:
 *   Phy address on success; NULL on failure.
 *
 ****************************************************************************/

uintptr_t up_addrenv_va_to_pa(FAR void *va)
{
  /* This cannot translate anything */

  return (uintptr_t)NULL;
}

#endif /* CONFIG_BUILD_KERNEL */
