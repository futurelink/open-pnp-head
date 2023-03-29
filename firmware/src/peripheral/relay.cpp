/*
  relays.cpp - relays & valves specific functions
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

#include "peripheral/relay.h"
#include "stm32/stm32_routines.h"

void Relay::init() {
    stm32_relay_init();
    stm32_relay_set_state(0x00);
}

void Relay::set_state(uint8_t state) {
    stm32_relay_set_state(state);
}

uint8_t Relay::get_state() const {
    auto pin_state = stm32_relay_get_state();
    return ((pin_state & (1 << RELAY_0_BIT)) ? 0b0001 : 0) |
           ((pin_state & (1 << RELAY_1_BIT)) ? 0b0010 : 0) |
           ((pin_state & (1 << RELAY_2_BIT)) ? 0b0100 : 0) |
           ((pin_state & (1 << RELAY_3_BIT)) ? 0b1000 : 0);
}
