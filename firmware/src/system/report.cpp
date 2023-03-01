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
 void Report::print_state(Vacuum *vacuum, uint8_t end_stops_state) {
    serial->print_string("<");
    auto st = state->get_state() & ~STATE_CYCLE_STOP;
    switch (st) {
        case STATE_IDLE: serial->print_string("Idle"); break;
        case STATE_CYCLE_PICK: serial->print_string("Pick"); break;
        case STATE_CYCLE_PLACE: serial->print_string("Place"); break;
        case STATE_CYCLE_ROTATE: serial->print_string("Rotate"); break;
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

    serial->print_string("|V:");
    for (uint8_t vac = 0; vac < VAC_SENSORS_N; vac++) {
        serial->print_float(vacuum->get_value(vac), 4);
        if (vac < VAC_SENSORS_N - 1) serial->write(',');
    }

    if (end_stops_state) {
        serial->print_string("|Pn:");
        if ((end_stops_state & (1 << 0)) != 0) serial->write('A');
        if ((end_stops_state & (1 << 1)) != 0) serial->write('B');
        if ((end_stops_state & (1 << 2)) != 0) serial->write('C');
        if ((end_stops_state & (1 << 3)) != 0) serial->write('D');
    }

    //serial->print_string("|F:");
    //serial->print_float(steppers->get_realtime_rate(), 1);

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
    if (digit_b) { serial->write(digit_b); }
    if (digit_a) { serial->write(digit_a); }
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
