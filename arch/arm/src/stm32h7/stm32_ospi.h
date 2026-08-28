/****************************************************************************
 * arch/arm/src/stm32h7/stm32_ospi.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_OSPI_H
#define __ARCH_ARM_SRC_STM32H7_STM32_OSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/qspi.h>

#include "chip.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ospi_initialize
 *
 * Description:
 *   Initialize the selected OCTOSPI port (1 or 2) and return a
 *   QSPI device instance usable with the NuttX QSPI interface
 *   (see include/nuttx/spi/qspi.h).
 *
 *   The board is responsible for configuring the OCTOSPI pins (the
 *   GPIO_OCTOSPI1_xxx / GPIO_OCTOSPI2_xxx definitions in the board.h)
 *   before calling this function, and for selecting the OCTOSPI kernel
 *   clock source via BOARD_OSPI_CLK (defaults to D1HCLK = 240 MHz on
 *   this port).
 *
 * Input Parameters:
 *   intf - OCTOSPI controller number (1 or 2)
 *
 * Returned Value:
 *   Valid QSPI device structure reference on success; NULL on failure
 *
 ****************************************************************************/

FAR struct qspi_dev_s *stm32_ospi_initialize(int intf);

#endif /* __ARCH_ARM_SRC_STM32H7_STM32_OSPI_H */
