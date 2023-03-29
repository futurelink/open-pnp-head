/*
  endstops.cpp - end stop specific functions
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

#include "peripheral/endstops.h"
#include "stm32/stm32_routines.h"

void EndStops::init() {
    stm32_limits_init();
}

void EndStops::interrupt() {

}

uint8_t EndStops::get_state() {
    auto pin_state = stm32_limits_get_state();
    return ((pin_state & (1 << LIMIT_0_BIT)) ? 0b0001 : 0) |
            ((pin_state & (1 << LIMIT_1_BIT)) ? 0b0010 : 0) |
            ((pin_state & (1 << LIMIT_2_BIT)) ? 0b0100 : 0) |
            ((pin_state & (1 << LIMIT_3_BIT)) ? 0b1000 : 0);
}

/**
 * Return true if all end-stops are at initial position.
 *
 * @return
 */
bool EndStops::all_at_zero() {
    return (get_state() == 0);
}
