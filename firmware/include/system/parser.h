/*
  parser.h
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
    uint8_t         sensor;         // Vacuum sensor number
    uint32_t        light_color;
} parser_state_t;

#endif // PARSER_H
