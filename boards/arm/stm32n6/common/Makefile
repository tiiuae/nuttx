#############################################################################
# boards/arm/stm32n6/common/Makefile
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
#############################################################################

include $(TOPDIR)/Make.defs

STM32_BOARD_COMMON_DIR := $(TOPDIR)$(DELIM)boards$(DELIM)arm$(DELIM)stm32$(DELIM)common

include board/Make.defs
include $(STM32_BOARD_COMMON_DIR)$(DELIM)src$(DELIM)Make.defs

DEPPATH += --dep-path board

ifeq ($(CONFIG_ARCH_BOARD_COMMON),y)
DEPPATH += --dep-path $(STM32_BOARD_COMMON_DIR)$(DELIM)src
VPATH += :$(STM32_BOARD_COMMON_DIR)$(DELIM)src
endif

include $(TOPDIR)/boards/Board.mk

CFLAGS += ${INCDIR_PREFIX}$(STM32_BOARD_COMMON_DIR)$(DELIM)include
