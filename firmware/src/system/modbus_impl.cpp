/*
  modbus_impl.cpp - ModBus related class
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

#include "system/modbus_impl.h"

#define RX_BUFFER_LEN                   64
#define TX_BUFFER_LEN                   64

ModBusSlave::ModBusSlave(uint8_t address, Settings *settings, State *state)
    : ModBus(address, RX_BUFFER_LEN, TX_BUFFER_LEN) {
    this->settings = settings;
    this->state = state;
}

uint8_t ModBusSlave::set_bit_reg_value(uint16_t reg, bool value) {
    if ((reg >= REGISTER_RELAYS) && (reg < REGISTER_RELAYS + 4)) {
        if (value) state->params.relays |= (1 << (reg - REGISTER_RELAYS)) /* offset */;
        else state->params.relays &= ~(1 << (reg - REGISTER_RELAYS) /* offset */);
        state->set_state(STATE_CYCLE_RELAY);
        return 0;
    }
    return EXCEPTION_ILLEGAL_ADDRESS; // Illegal Data Address exception code
}

bool ModBusSlave::get_bit_reg_value(uint16_t reg) {
    if ((reg >= REGISTER_RELAYS) && (reg < REGISTER_RELAYS + 4))
        return (state->relays & (1 << (reg - REGISTER_RELAYS))) != 0;
    else if ((reg >= REGISTER_END_STOPS) && (reg < REGISTER_END_STOPS + 4))
        return (state->end_stops & (1 << (reg - REGISTER_END_STOPS))) != 0;
    return false;
}

uint8_t ModBusSlave::set_16bit_reg_value(uint16_t reg, uint16_t value) {
    if (reg == REGISTER_LIGHT_MSW) {
        state->params.light_color = (state->params.light_color & 0x0000ffff) | (value << 16);
        state->set_state(STATE_CYCLE_LIGHT);
        return 0;
    }
    else if (reg == REGISTER_LIGHT_LSW) {
        state->params.light_color = (state->params.light_color & 0xffff0000) | value;
        state->set_state(STATE_CYCLE_LIGHT);
        return 0;
    }
    else if ((reg >= REGISTER_POSITION_BASE) && (reg < REGISTER_POSITION_BASE + NOZZLE_N * 4)) {
        // Can not set position while status is not IDLE
        if (!state->is_state(STATE_IDLE)) return EXCEPTION_BUSY;  // Slave Device Busy exception code

        uint8_t nozzle = (reg - REGISTER_POSITION_BASE) / 2;
        if (nozzle < ROTARY_AXIS_N) {
            if ((reg - REGISTER_POSITION_BASE) % 2) ((uint16_t *) &state->params.angle)[1] = value; // Write LS 16-bit word
            else {
                state->params.nozzle = nozzle;
                state->params.feed = 3000;
                ((uint16_t *) &state->params.angle)[0] = value;                                     // Write MS 16-bit word
                state->set_state(STATE_CYCLE_ROTATE);                                               // and rotate
            }
        } else {
            if ((reg - REGISTER_POSITION_BASE) % 2) ((uint16_t *) &state->params.depth)[1] = value; // Write LS 16-bit word
            else {
                state->params.nozzle = nozzle - ROTARY_AXIS_N;
                state->params.feed = 3000;
                ((uint16_t *) &state->params.depth)[0] = value;                                     // Write MS 16-bit word
                state->set_state(STATE_CYCLE_MOVE);                                                 // and move
            }
        }
        return 0;
    }

    return EXCEPTION_ILLEGAL_ADDRESS; // Illegal Data Address exception code
}

uint16_t ModBusSlave::get_16bit_reg_value(uint16_t reg) {
    if (reg == REGISTER_STATE) return state->get_state();

    else if (reg == REGISTER_LIGHT_MSW) return (state->params.light_color >> 16) & 0xffff;
    else if (reg == REGISTER_LIGHT_LSW) return state->params.light_color & 0xffff;

        // Vacuum is 32-bit register that holds float value, so we send MS 16-bit word in REGISTER_VAC_SENSOR_n and
        // LS 16-bit word in REGISTER_VAC_SENSOR_n+1
    else if ((reg >= REGISTER_VAC_SENSOR_BASE) && (reg < REGISTER_VAC_SENSOR_BASE + NOZZLE_N * 2)) {
        float vac = state->get_vacuum((reg - REGISTER_VAC_SENSOR_BASE) / 2);
        if ((reg - REGISTER_VAC_SENSOR_BASE) % 2) return ((uint16_t *) &vac)[1];    // Send LS 16-bit word
        else return ((uint16_t *) &vac)[0];                                         // Send MS 16-bit word
    }

        // Get axis position - it fits into 2 16-bit registers
    else if ((reg >= REGISTER_POSITION_BASE) && (reg < REGISTER_POSITION_BASE + NOZZLE_N * 4)) {
        uint8_t nozzle = (reg - REGISTER_POSITION_BASE) / 2;

        float pos = 0;
        if (nozzle < ROTARY_AXIS_N) pos = (float) state->get_positions()[nozzle] / settings->get_steps_per_degree(nozzle);
        else {
            nozzle -= ROTARY_AXIS_N;
            uint8_t axis = ROTARY_AXIS_N + nozzle / 2;
            pos = settings->convert_linear_steps_to_mm(state->get_positions()[axis]) * ((nozzle % 2) ? 1.0f : -1.0f);
        }

        if ((reg - REGISTER_POSITION_BASE) % 2) return ((uint16_t *) &pos)[1];      // Send LS 16-bit word
        else return ((uint16_t *) &pos)[0];                                         // Send MS 16-bit word
    }

    return 0;
}
