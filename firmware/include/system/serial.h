/*
  serial.h
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

#ifndef SERIAL_H
#define SERIAL_H

#include <cstdint>

#define SERIAL_NO_DATA  0xff
#define RX_BUFFER_SIZE  128
#define TX_BUFFER_SIZE  128

class Serial {
private:
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint8_t rx_buffer_head;
    volatile uint8_t rx_buffer_tail;

    uint8_t tx_buffer[TX_BUFFER_SIZE];
    uint8_t tx_buffer_head;
    volatile uint8_t tx_buffer_tail;

public:
    explicit Serial();

    void init();

    // Writes one byte to the TX serial buffer. Called by main program.
    void write(uint8_t data);

    // Fetches the first byte in the serial read buffer. Called by main program.
    uint8_t read();

    // Reset and empty data in read buffer. Used by e-stop and reset.
    void reset_read_buffer();

    // Returns the number of bytes available in the RX serial buffer.
    uint8_t get_rx_buffer_available() const;

    void receive();
    void transmit();

    void print_string(const char *s);
    void print_integer(int32_t n);
    void print_uint32_base10(uint32_t n);
    void print_float(float n, uint8_t decimal_places);
};


#endif // SERIAL_H
