#include <cstdint>

#ifndef PARSER_H
#define PARSER_H

#define WORD_A      0
#define WORD_Z      1

typedef struct line_state_t {
    char        letter;
    float       value;
    uint8_t     int_value;
    uint16_t    mantissa;
    uint8_t     command;
    uint8_t     word_bits;
} line_state_t;

typedef struct parser_state_t {
    line_state_t    line_state;
    uint8_t         relay;          // {M100-104,M110-114}
    uint8_t         nozzle;         // Pick-Place active nozzle
    float           depth;          // Pick-Place cycle var indicates how deep the nozzle should go
    float           angle;          // Pick-Place rotate cycle var indicates rotation angle
} parser_state_t;

#endif // PARSER_H
