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

    bool transmit_buf(uint8_t *buf, uint8_t len);

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
