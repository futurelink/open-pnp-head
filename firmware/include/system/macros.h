/*
  macros.h - macro definitions
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

#ifndef MACROS_H
#define MACROS_H

#include "config.h"
#include "cpu_map.h"

#define MAX_INT_DIGITS 6

#define max(a, b)               (((a) > (b)) ? (a) : (b))
#define min(a, b)               (((a) < (b)) ? (a) : (b))

#define TICKS_PER_MICROSECOND       (F_CPU / 1000000UL)
#define DT_SEGMENT                  (1.0f / (ACCELERATION_TICKS_PER_SECOND * 60.0f)) // min/segment
#define REQ_MM_INCREMENT_SCALAR     1.25f

#define DISABLE_IRQ             __disable_irq();
#define ENABLE_IRQ              __enable_irq();

#endif // MACROS_H
