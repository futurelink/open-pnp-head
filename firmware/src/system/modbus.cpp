/*
  modbus.cpp - ModBus related class
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

#include "system/modbus.h"
#include "stm32/stm32_routines.h"
#include <cmath>

ModBus::ModBus(uint8_t address, Settings *settings, State *state) {
    this->address = address;
    this->settings = settings;
    this->state = state;
    this->rx_len = 0;
    this->tx_len = 0;
}

void ModBus::init() {
    stm32_rs485_init();
    stm32_rs485_silence_timer_init();
    stm32_rs485_silence_timer_reset();
}

void ModBus::receive() {
    uint8_t byte = 0;
    stm32_rs485_silence_timer_reset();      // Reset timeout
    if (stm32_rs485_receive_byte(&byte)) {
        if (rx_len < RX_BUFFER_LEN) rx_buffer[rx_len++] = byte; // Receive byte
    }
}

void ModBus::transmit() {
    if (tx_len > 0) {
        for (uint8_t i = 0; i < tx_len; i++) stm32_rs485_transmit_byte(tx_buffer[i]);
        stm32_rs485_stop_transmission();
        tx_len = 0;
    }
}

void ModBus::silence() {
    if (rx_len > 0) {
        process_function();
        rx_len = 0;
    }
}

void ModBus::process_function() {
    if ((rx_len == 0) || (rx_buffer[0] != address)) return;
    switch (rx_buffer[1]) {
        case FUNCTION_READ_COIL:
        case FUNCTION_READ_INPUT:
        case FUNCTION_READ_HOLDING_REGISTER:
        case FUNCTION_READ_INPUT_REGISTER:
        case FUNCTION_WRITE_COIL:
        case FUNCTION_WRITE_HOLDING_REGISTER:
            if (rx_len == 8) {
                uint16_t reg_base = 0 + ((rx_buffer[2] << 8) | (rx_buffer[3]));
                uint16_t reg_number = (rx_buffer[4] << 8) | rx_buffer[5];
                if (crc16(rx_buffer, rx_len - 2) == ((rx_buffer[7] << 8) | rx_buffer[6])) {
                    switch (rx_buffer[1]) { // CRC is OK
                        case FUNCTION_READ_COIL: send_coil_status(reg_base, reg_number); break;
                        case FUNCTION_READ_INPUT: send_input_status(reg_base, reg_number); break;
                        case FUNCTION_READ_HOLDING_REGISTER: send_holding_registers(reg_base, reg_number); break;
                        case FUNCTION_READ_INPUT_REGISTER: send_analog_registers(reg_base, reg_number); break;
                        case FUNCTION_WRITE_COIL: write_single_coil(reg_base, reg_number /* value: 0xff00 or 0x0000 */); break;
                        case FUNCTION_WRITE_HOLDING_REGISTER: write_single_register(reg_base, reg_number /* value */); break;
                    }
                }
            }
            break;

        case 15: // Write multiple coils
        case 16: // Write multiple registers
            break;
        default: send_exception(rx_buffer[1], 0x01); break;
    }
}

void ModBus::send_coil_status(uint16_t reg_base, uint16_t reg_number) {
    uint8_t len = 3;
    tx_buffer[0] = address;
    tx_buffer[1] = FUNCTION_READ_COIL;
    tx_buffer[2] = (uint8_t) ceilf((float) reg_number / 8);

    // add coil status bits
    uint8_t byte = 0, i = 0;
    while (i < reg_number) {
        if (get_bit_reg_value(REGISTER_COIL_BASE + reg_base + i)) tx_buffer[3 + byte] |= (1 << (i % 8));
        else tx_buffer[3 + byte] &= ~(1 << (i % 8));
        if (++i % 8 == 0) { byte++; len++; }
    }
    add_crc_and_send(len);
}

void ModBus::send_input_status(uint16_t reg_base, uint16_t reg_number) {
    uint8_t len = 3;
    tx_buffer[0] = address;
    tx_buffer[1] = FUNCTION_READ_INPUT;
    tx_buffer[2] = (uint8_t) ceilf((float) reg_number / 8);

    // add input status bits
    uint8_t byte = 0, i = 0;
    while (i < reg_number) {
        if (get_bit_reg_value(REGISTER_INPUT_BASE + reg_base + i)) tx_buffer[3 + byte] |= (1 << (i % 8));
        else tx_buffer[3 + byte] &= ~(1 << (i % 8));
        if (++i % 8 == 0) { byte++; len++; }
    }
    add_crc_and_send(len);
}

void ModBus::send_holding_registers(uint16_t reg_base, uint16_t reg_number) {
    uint8_t len = 3;
    tx_buffer[0] = address;
    tx_buffer[1] = FUNCTION_READ_HOLDING_REGISTER;
    tx_buffer[2] = reg_number * 2;

    // add registers
    for (uint16_t i = 0; i < reg_number; i++) {
        uint16_t value = get_16bit_reg_value(REGISTER_HOLDING_BASE + reg_base + i);
        tx_buffer[3 + i * 2] = (value >> 8) & 0xff;
        tx_buffer[4 + i * 2] = value & 0xff;
        len += 2;
    }
    add_crc_and_send(len);
}

void ModBus::send_analog_registers(uint16_t reg_base, uint16_t reg_number) {
    uint8_t len = 3;
    tx_buffer[0] = address;
    tx_buffer[1] = FUNCTION_READ_INPUT_REGISTER;
    tx_buffer[2] = reg_number * 2;

    // add analog input registers
    for (uint16_t i = 0; i < reg_number; i++) {
        uint16_t value = get_16bit_reg_value(REGISTER_ANALOG_INPUT_BASE + reg_base + i);
        tx_buffer[3 + i * 2] = (value >> 8) & 0xff;
        tx_buffer[4 + i * 2] = value & 0xff;
        len += 2;
    }
    add_crc_and_send(len);
}

uint8_t ModBus::set_bit_reg_value(uint16_t reg, bool value) {
    if ((reg >= REGISTER_RELAYS) && (reg < REGISTER_RELAYS + 4)) {
        if (value) state->params.relays |= (1 << (reg - REGISTER_RELAYS)) /* offset */;
        else state->params.relays &= ~(1 << (reg - REGISTER_RELAYS) /* offset */);
        state->set_state(STATE_CYCLE_RELAY);
        return 0;
    }
    return 0x02; // Illegal Data Address exception code
}

bool ModBus::get_bit_reg_value(uint16_t reg) {
    if ((reg >= REGISTER_RELAYS) && (reg < REGISTER_RELAYS + 4))
        return (state->relays & (1 << (reg - REGISTER_RELAYS))) != 0;
    else if ((reg >= REGISTER_END_STOPS) && (reg < REGISTER_END_STOPS + 4))
        return (state->end_stops & (1 << (reg - REGISTER_END_STOPS))) != 0;
    return false;
}

uint8_t ModBus::set_16bit_reg_value(uint16_t reg, uint16_t value) {
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
        if (!state->is_state(STATE_IDLE)) return 0x06;  // Slave Device Busy exception code

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

    return 0x02; // Illegal Data Address exception code
}

uint16_t ModBus::get_16bit_reg_value(uint16_t reg) {
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


void ModBus::send_exception(uint8_t function, uint8_t code) {
    tx_buffer[0] = address;
    tx_buffer[1] = function | 0x80;
    tx_buffer[2] = code;
    add_crc_and_send(3);
}

void ModBus::write_single_coil(uint16_t reg_base, uint16_t value) {
    if ((value == 0xff00) || (value == 0)) {
        tx_buffer[0] = address;
        tx_buffer[1] = FUNCTION_WRITE_COIL;
        uint8_t result = set_bit_reg_value(REGISTER_COIL_BASE + reg_base, (value == 0xff00));
        if (result == 0) {
            // Send response
            tx_buffer[2] = (reg_base >> 8) & 0xff;
            tx_buffer[3] = reg_base & 0xff;
            tx_buffer[4] = (value >> 8) & 0xff;
            tx_buffer[5] = value & 0xff;
            add_crc_and_send(6);
        } else {
            // Send exception
            tx_buffer[1] |= 0x80;
            tx_buffer[2] = result;
            add_crc_and_send(3);
        }
    }
}

void ModBus::write_single_register(uint16_t reg_base, uint16_t value) {
    uint8_t result = set_16bit_reg_value(REGISTER_HOLDING_BASE + reg_base, value);
    tx_buffer[0] = address;
    tx_buffer[1] = FUNCTION_WRITE_HOLDING_REGISTER;
    if (!result) {
        // Send response
        tx_buffer[2] = (reg_base >> 8) & 0xff;
        tx_buffer[3] = reg_base & 0xff;
        tx_buffer[4] = (value >> 8) & 0xff;
        tx_buffer[5] = value & 0xff;
        add_crc_and_send(6);
    } else {
        // Send exception
        tx_buffer[1] |= 0x80;
        tx_buffer[2] = result;
        add_crc_and_send(3);
    }
}

void ModBus::add_crc_and_send(uint8_t len) {
    uint16_t crc = crc16(tx_buffer, len);
    tx_buffer[len] = crc & 0xff;            // LSB
    tx_buffer[len+1] = (crc >> 8) & 0xff;   // MSB
    len += 2;

    // Send
    tx_len = len;
    stm32_rs485_start_transmission();
}

uint16_t ModBus::crc16(const unsigned char *buf, unsigned int len) {
    uint8_t xor_val;
    uint16_t crc = 0xFFFF;
    while(len--) {
        xor_val = (*buf++) ^ crc;
        crc >>= 8;
        crc ^= crc16_table[xor_val];
    }
    return crc;
}


