/*
  stm32_cpu_map.h
  Part of open-pnp-head

  Copyright (c) 2022 Denis Pavlov

  open-pnp-head is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  open-pnp-head is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with open-pnp-head.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STM32_CPU_MAP_H
#define STM32_CPU_MAP_H

#include "stm32f1xx_hal.h"

// Define step pulse output pins
// All step bit pins must be on the same port
#define STEP_PORT           GPIOB
#define ROTARY_1_STEP_BIT   10
#define ROTARY_2_STEP_BIT   11
#define ROTARY_3_STEP_BIT   12
#define ROTARY_4_STEP_BIT   13
#define ROTARY_STEP_MASK    ((1 << ROTARY_1_STEP_BIT) | (1 << ROTARY_2_STEP_BIT) | \
                            (1 << ROTARY_3_STEP_BIT) | (1 << ROTARY_4_STEP_BIT))
#define LINEAR_1_STEP_BIT   15
#define LINEAR_2_STEP_BIT   14
#define LINEAR_STEP_MASK    ((1 << LINEAR_1_STEP_BIT) | (1 << LINEAR_2_STEP_BIT))
#define STEP_MASK           (ROTARY_STEP_MASK | LINEAR_STEP_MASK)

// Define step direction output pins
// All direction pins must be on the same port
#define DIRECTION_PORT          GPIOB
#define ROTARY_1_DIR_BIT        4
#define ROTARY_2_DIR_BIT        5
#define ROTARY_3_DIR_BIT        6
#define ROTARY_4_DIR_BIT        7
#define ROTARY_DIRECTION_MASK  ((1 << ROTARY_1_DIR_BIT) | (1 << ROTARY_2_DIR_BIT) | \
                                (1 << ROTARY_3_DIR_BIT) | (1 << ROTARY_4_DIR_BIT))
#define LINEAR_1_DIR_BIT        9
#define LINEAR_2_DIR_BIT        8
#define LINEAR_DIR_MASK         ((1 << LINEAR_1_DIR_BIT) | (1 << LINEAR_2_DIR_BIT))
#define DIRECTION_MASK          (ROTARY_DIRECTION_MASK | LINEAR_DIR_MASK)

// Define stepper driver enable/disable output pin
#define STEPPERS_DISABLE_PORT       GPIOB
#define STEPPERS_DISABLE_BIT        3
#define STEPPERS_DISABLE_MASK       (1 << STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins and limit interrupt vectors
// NOTE: All limit bit pins must be on the same port
#define LIMIT_PORT       GPIOA
#define LIMIT_1_BIT      4
#define LIMIT_2_BIT      5
#define LIMIT_3_BIT      6
#define LIMIT_4_BIT      7
#define LIMIT_MASK       ((1 << LIMIT_1_BIT) | (1 << LIMIT_2_BIT) | (1 << LIMIT_3_BIT) | (1 << LIMIT_4_BIT))

#define RELAY_PORT      GPIOA
#define RELAY_0_BIT     8
#define RELAY_1_BIT     11
#define RELAY_2_BIT     12
#define RELAY_3_BIT     15

#define LIGHT_PORT      GPIOB
#define LIGHT_BIT       0

// IO port settings
#define RS485_RW_PORT   GPIOB
#define RS485_RW_BIT    1
#define RS485_PORT      GPIOA
#define RS485_TX_BIT    9
#define RS485_RX_BIT    10

#endif // STM32_CPU_MAP_H
