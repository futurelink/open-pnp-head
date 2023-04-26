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
#include <cstring>

ModBus::ModBus(uint8_t address, uint8_t rx_buffer_len, uint8_t tx_buffer_len) {
    this->address = address;
    this->rx_buffer_len = rx_buffer_len;
    this->rx_buffer = (uint8_t *) malloc(rx_buffer_len);
    this->tx_buffer = (uint8_t *) malloc(tx_buffer_len);
    this->rx_len = 0;
    this->tx_len = 0;
}

ModBus::~ModBus() {
    free(rx_buffer);
    free(tx_buffer);
}

void ModBus::init() {
    stm32_rs485_init();
    stm32_rs485_silence_timer_init((uint16_t) BAUD_RATE);
    stm32_rs485_silence_timer_reset();
    this->rx_len = 0;
    this->tx_len = 0;
}

void ModBus::receive() {
    uint8_t byte = 0;
    stm32_rs485_silence_timer_reset();      // Reset timeout
    if (stm32_rs485_receive_byte(&byte)) {
        if (rx_len < rx_buffer_len) rx_buffer[rx_len++] = byte; // Receive byte
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
        if (rx_buffer[0] != address) return;
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

            case FUNCTION_WRITE_MULTI_COILS: // Not implemented yet
                send_exception(rx_buffer[1], EXCEPTION_ILLEGAL_FUNCTION);
                break;

            case FUNCTION_WRITE_MULTI_REGISTERS:
                if (rx_len > 8) {
                    uint16_t reg_base = 0 + ((rx_buffer[2] << 8) | (rx_buffer[3]));
                    uint16_t reg_number = (rx_buffer[4] << 8) | rx_buffer[5];
                    uint8_t data_bytes = rx_buffer[6];
                    if (data_bytes + 6 > rx_len) { send_exception(rx_buffer[1], EXCEPTION_ILLEGAL_ADDRESS); break; } // buffer overflow
                    if (crc16(rx_buffer, rx_len - 2) == ((rx_buffer[rx_len - 1] << 8) | rx_buffer[rx_len - 2])) {
                        switch (rx_buffer[1]) { // CRC is OK
                            case FUNCTION_WRITE_MULTI_REGISTERS:
#ifdef MULTI_WRITE_MSW_FIRST
                                for (int i = reg_number - 1; i >= 0; i--) {
#else
                                for (int i = 0; i < reg_number ; i++) {
#endif
                                    write_single_register(reg_base + i, (rx_buffer[7 + i * 2] << 8) | rx_buffer[8 + i * 2]);
                                }
                                memcpy(tx_buffer, rx_buffer, 6);
                                add_crc_and_send(6);
                                break;
                        }
                    }
                }
                break;

            default: send_exception(rx_buffer[1], EXCEPTION_ILLEGAL_FUNCTION); break;
        }

        rx_len = 0;
    }
}

void ModBus::send_coil_status(uint16_t reg_base, uint16_t reg_number) {
    uint8_t len = 3;
    tx_buffer[0] = address;
    tx_buffer[1] = FUNCTION_READ_COIL;
    tx_buffer[2] = (uint8_t) ceilf((float) reg_number / 8);
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
    for (uint16_t i = 0; i < reg_number; i++) {
        uint16_t value = get_16bit_reg_value(REGISTER_ANALOG_INPUT_BASE + reg_base + i);
        tx_buffer[3 + i * 2] = (value >> 8) & 0xff;
        tx_buffer[4 + i * 2] = value & 0xff;
        len += 2;
    }
    add_crc_and_send(len);
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
            tx_buffer[2] = (reg_base >> 8) & 0xff;
            tx_buffer[3] = reg_base & 0xff;
            tx_buffer[4] = (value >> 8) & 0xff;
            tx_buffer[5] = value & 0xff;
            add_crc_and_send(6); // Send response
        } else
            send_exception(FUNCTION_WRITE_COIL, result);
    }
}

void ModBus::write_single_register(uint16_t reg_base, uint16_t value) {
    uint8_t result = set_16bit_reg_value(REGISTER_HOLDING_BASE + reg_base, value);
    tx_buffer[0] = address;
    tx_buffer[1] = FUNCTION_WRITE_HOLDING_REGISTER;
    if (!result) {
        tx_buffer[2] = (reg_base >> 8) & 0xff;
        tx_buffer[3] = reg_base & 0xff;
        tx_buffer[4] = (value >> 8) & 0xff;
        tx_buffer[5] = value & 0xff;
        add_crc_and_send(6); // Send response
    } else
        send_exception(FUNCTION_WRITE_HOLDING_REGISTER, result);
}

void ModBus::add_crc_and_send(uint8_t len) {
    uint16_t crc = crc16(tx_buffer, len);
    tx_buffer[len] = crc & 0xff;            // LSB
    tx_buffer[len+1] = (crc >> 8) & 0xff;   // MSB
    tx_len = len + 2;
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
