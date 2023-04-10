/*
  report.cpp - reporting routines
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

#include "system/report.h"

#include <cstdlib>

Report::Report(Settings *settings, Serial *serial, State *state) {
    this->serial = serial;
    this->state = state;
    this->settings = settings;
}

/**
 * Report current machine state and substates
 * @param st
  */
 void Report::print_state(Vacuum *vacuum, uint8_t end_stops_state, uint8_t relay_state, uint32_t light_state) {
    serial->print_string("<");
    auto st = state->get_state() & ~STATE_CYCLE_STOP;
    switch (st) {
        case STATE_IDLE: serial->print_string("Idle"); break;
        case STATE_CYCLE_PICK: serial->print_string("Pick"); break;
        case STATE_CYCLE_PLACE: serial->print_string("Place"); break;
        case STATE_CYCLE_ROTATE: serial->print_string("Rotate"); break;
        case STATE_CYCLE_MOVE: serial->print_string("Move"); break;
        case STATE_HOMING: serial->print_string("Home"); break;
        case STATE_ALARM: serial->print_string("Alarm"); break;
        default: break;
    }

    auto print_position = (float *) malloc(sizeof(float) * ROTARY_AXIS_N);
    settings->convert_rotary_steps_to_degrees(print_position, state->get_positions(), ROTARY_AXIS_N);
    serial->print_string("|RPos:");
    print_axis_values(print_position, ROTARY_AXIS_N);
    free(print_position);

    print_position = (float *) malloc(sizeof(float) * LINEAR_AXIS_N);
    settings->convert_linear_steps_to_mm(print_position, state->get_positions() + ROTARY_AXIS_N, LINEAR_AXIS_N);
    serial->print_string("|LPos:");
    print_axis_values(print_position, LINEAR_AXIS_N);
    free(print_position);

#ifdef VAC_SENSORS_N
    serial->print_string("|V:");
    for (uint8_t vac = 0; vac < VAC_SENSORS_N; vac++) {
        serial->print_float(vacuum->get_value(vac), 4);
        if (vac < VAC_SENSORS_N - 1) serial->write(',');
    }
#endif

    if (relay_state) {
        serial->print_string("|R:");
        for (uint8_t idx = 0; idx < RELAY_N; idx++) {
            if ((relay_state & (1 << idx)) != 0) serial->write('A' + idx);
        }
    }

    if (light_state) {
        serial->print_string("|L:");
#ifdef WS8212LED
        serial->print_integer((int32_t)(light_state & 0xff));           // Red
        serial->print_string(",");
        serial->print_integer((int32_t)(light_state & 0xff00) >> 8);    // Green
        serial->print_string(",");
        serial->print_integer((int32_t)(light_state & 0xff0000) >> 16); // Blue
#else
        serial->print_string("0");
#endif
    }

    if (end_stops_state) {
        serial->print_string("|Pn:");
        for (uint8_t idx = 0; idx < ROTARY_AXIS_N; idx++) {
            if ((end_stops_state & (1 << idx)) != 0) serial->write('A' + idx);
        }
    }

    serial->print_string(">\n");
}

void Report::print_line_feed() {
    serial->print_string("\r\n");
}

void Report::print_axis_values(const float *values, uint8_t n_axis) {
    for (uint8_t idx = 0; idx < n_axis; idx++) {
        serial->print_float(values[idx],3);
        if (idx < (n_axis - 1)) serial->write(',');
    }
}

void Report::print_float(float val) {
     serial->print_float(val, 4);
 }

/**
 * Prints an uint8 variable in base 10.
 * @param n
 */
void Report::print_uint8_base10(uint8_t n) {
    uint8_t digit_a = 0;
    uint8_t digit_b = 0;
    if (n >= 100) { // 100-255
        digit_a = '0' + n % 10;
        n /= 10;
    }

    if (n >= 10) { // 10-99
        digit_b = '0' + n % 10;
        n /= 10;
    }

    serial->write('0' + n);
    if (digit_b) serial->write(digit_b);
    if (digit_a) serial->write(digit_a);
}

void Report::status_message(uint8_t code) {
    if (code == 0) {
        serial->print_string("ok\r\n");
    } else {
        serial->print_string("error:");
        print_uint8_base10(code);
        print_line_feed();
    }
}
