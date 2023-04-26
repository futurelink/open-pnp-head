/*
  state.cpp - FSM state storage
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

#include "system/state.h"

#include <cstring>
#include <cstdlib>

State::State() {
    this->state = STATE_IDLE;
    this->relays = 0;
    this->light_color = 0;
    this->pick_place_state = 0;
    this->positions = (int32_t *) malloc(sizeof(int32_t) * AXIS_N);
}

void State::init() {
    this->state = STATE_IDLE;
    this->pick_place_state = 0;
    memset(this->positions, 0, sizeof(int32_t) * AXIS_N);
    memset(&this->params, 0, sizeof(fsm_params_t));
    set_end_motion();
}
