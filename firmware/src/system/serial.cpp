/*
  serial.cpp - serial port interaction routines
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

#include "system/serial.h"
#include "stm32/stm32_routines.h"
#include "system/macros.h"

Serial::Serial() {
    rx_buffer_tail = rx_buffer_head = 0;
    tx_buffer_tail = tx_buffer_head = 0;
}

void Serial::init() {
    stm32_rs485_init();
}

/**
 * Returns the number of bytes available in the RX serial buffer.
 * @return
 */
uint8_t Serial::get_rx_buffer_available() const {
    auto r_tail = rx_buffer_tail; // Copy to limit multiple calls to volatile
    if (rx_buffer_head >= r_tail) return RX_BUFFER_SIZE - (rx_buffer_head - r_tail);
    return r_tail - rx_buffer_head - 1;
}

/**
 * Writes one byte to the transmit serial buffer
 * @param data
 */
void Serial::write(uint8_t data) {
    uint8_t next_head = tx_buffer_head + 1; // Calculate next head
    if (next_head == TX_BUFFER_SIZE) next_head = 0;

    // Wait until there is space in the buffer
    while (next_head == tx_buffer_tail) asm volatile ("nop");

    // Store data and advance head
    tx_buffer[tx_buffer_head] = data;
    tx_buffer_head = next_head;

    stm32_rs485_start_transmission();
}

/**
 * Fetches the first byte in the serial read buffer
 */
uint8_t Serial::read() {
    auto tail = rx_buffer_tail; // Temporary rx_buffer_tail (to optimize for volatile)
    if (rx_buffer_head == tail) return SERIAL_NO_DATA;

    auto data = rx_buffer[tail];
    if (++tail == RX_BUFFER_SIZE) tail = 0;
    rx_buffer_tail = tail;

    return data;
}

void Serial::reset_read_buffer() {
    rx_buffer_tail = rx_buffer_head;
}

void Serial::print_string(const char *s) {
    while (*s) write(*s++);
}

void Serial::print_integer(int32_t n) {
    if (n < 0) {
        write('-');
        print_uint32_base10(-n);
    } else {
        print_uint32_base10(n);
    }
}

void Serial::print_uint32_base10(uint32_t n) {
    if (n == 0) {
        write('0');
        return;
    }

    unsigned char buf[10];
    uint8_t i = 0;

    while (n > 0) {
        buf[i++] = n % 10;
        n /= 10;
    }

    for (; i > 0; i--) write('0' + buf[i-1]);
}

void Serial::print_float(float n, uint8_t decimal_places) {
    if (n < 0) { write('-'); n = -n; }

    uint8_t decimals = decimal_places;
    while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
        n *= 100;
        decimals -= 2;
    }
    if (decimals) { n *= 10; }
    n += 0.5; // Add rounding factor. Ensures carryover through entire value.

    // Generate digits backwards and store in string.
    unsigned char buf[13];
    uint8_t i = 0;
    uint32_t a = (long)n;
    while(a > 0) {
        buf[i++] = (a % 10) + '0'; // Get digit
        a /= 10;
    }
    while (i < decimal_places) {
        buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
    }
    if (i == decimal_places) { // Fill in leading zero, if needed.
        buf[i++] = '0';
    }

    // Print the generated string.
    for (; i > 0; i--) {
        if (i == decimal_places) { write('.'); } // Insert decimal point in right place.
        write(buf[i-1]);
    }
}

/**
 * Sends data to serial port.
 */
void Serial::transmit() {
    if (tx_buffer_head == tx_buffer_tail) {
        stm32_rs485_stop_transmission();
        return; // Nothing to send, bail.
    }

    //                 * - data to be sent
    //                 -------------------------------------------------------------------------------------------------
    // if head > tail:                                     Tail ************************ Head
    // if head < tail: ******************* Head                               Tail *************************************
    if (tx_buffer_head > tx_buffer_tail) {
        if (!stm32_rs485_transmit_byte(tx_buffer[tx_buffer_tail])) return;
        tx_buffer_tail++;
    } else if (tx_buffer_head < tx_buffer_tail) {
        // Send data to end of the buffer and move tail to buffer start
        if (tx_buffer_tail < TX_BUFFER_SIZE) {
            if (!stm32_rs485_transmit_byte(tx_buffer[tx_buffer_tail])) return;
            tx_buffer_tail++;
        } else tx_buffer_tail = 0;
    }
}

/**
 * Receivse data from serial port.
 * @param data
 */
void Serial::receive() {
    uint8_t byte = 0;
    if (stm32_rs485_receive_byte(&byte)) {
        uint8_t next_head = rx_buffer_head + 1;
        if (next_head == RX_BUFFER_SIZE) next_head = 0;

        // Write data to buffer unless it is full
        if (next_head != rx_buffer_tail) {
            rx_buffer[rx_buffer_head] = byte;
            rx_buffer_head = next_head;
        }
    }
}
