#include <cstdint>

#ifndef PARSER_H
#define PARSER_H

#define WORD_A      0
#define WORD_B      1
#define WORD_F      2
#define WORD_G      3
#define WORD_R      4
#define WORD_Z      5

typedef struct line_state_t {
    char        letter;
    float       value;
    uint8_t     int_value;
    uint16_t    mantissa;
    uint8_t     command;
    uint8_t     word_bits;
    uint32_t    light_color;
} line_state_t;

typedef struct parser_state_t {
    line_state_t    line_state;
    uint8_t         relay;          // {M100-104,M110-114}
    uint8_t         nozzle;         // Pick-Place active nozzle
    float           depth;          // Pick-Place cycle var indicates how deep the nozzle should go
    float           angle;          // Pick-Place rotate cycle var indicates rotation angle
    float           feed;           // Pick-Place or Move cycle feed rate
    uint32_t        light_color;
} parser_state_t;

#endif // PARSER_H
