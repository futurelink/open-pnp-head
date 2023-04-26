/*
  text_control.cpp - text console control routines
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

#include <cstring>

#include "system/text_control.h"
#include "functions.h"

TextControl::TextControl(Settings *settings, Report *report, Motion *motion, State *state, Callbacks *callbacks) :
    Control(settings, motion, state, callbacks) {
    this->report = report;
    this->parser_state = {};
}

void TextControl::init() {
    Control::init();
    memset(&parser_state, 0, sizeof(parser_state_t));
}

uint8_t TextControl::parse_serial_input() {
    uint8_t result = STATUS_OK;
    uint8_t char_counter = 0;

    // Initialize line state
    auto line_state = &parser_state.line_state;
    line_state->command = COMMAND_NONE;
    line_state->word_bits = 0;
    line_state->letter = 0;
    line_state->light_color = 0;

    // Parse the line
    while (line[char_counter] != 0) {
        line_state->letter = line[char_counter];
        if ((line_state->letter < 'A') || (line_state->letter > 'Z')) return STATUS_EXPECTED_COMMAND_LETTER;

        char_counter++;
        if (!Functions::read_float(line, &char_counter, &line_state->value)) return STATUS_BAD_NUMBER_FORMAT;

        // Get integer value for command code (e.g. M5, P10)
        line_state->int_value = (uint8_t) truncf(line_state->value);
        line_state->mantissa = (uint16_t) lroundf(100 * (line_state->value - (float) line_state->int_value));

        // Pick command T[nozzle] Z[depth], i.e. P0 Z-10, P1 Z-5, etc.
        // Place command the same with 'P' i.e. L0 Z-8, L1 Z-10, etc
        // Rotate command R[nozzle] A[degree] i.e. R1 A90
        // Go command G[nozzle] Z[depth]
        // Valve relay command V[0-4]
        if (line_state->command) {
            result = process_parameter();
        } else {
            switch(parser_state.line_state.letter) {
                // 1st modal group
                case 'R': line_state->command = COMMAND_ROTATE; parser_state.nozzle = line_state->int_value; break;
                case 'T': line_state->command = COMMAND_PICK; parser_state.nozzle = line_state->int_value; break;
                case 'P': line_state->command = COMMAND_PLACE; parser_state.nozzle = line_state->int_value; break;
                case 'G': line_state->command = COMMAND_MOVE; parser_state.nozzle = line_state->int_value; break;
                case 'S': line_state->command = COMMAND_READ_SENSOR; parser_state.sensor = line_state->int_value; break;

                // 2nd modal group
                case 'L': line_state->command = COMMAND_LIGHT; break;

                //  3rd modal group
                case 'V': line_state->command = COMMAND_RELAY; result = process_relay(); break;
                default: result = STATUS_CODE_UNSUPPORTED_COMMAND;
            }
        }
    }

    // Command must have valid nozzle number
    if (line_state->command) {
        if ((parser_state.nozzle > ROTARY_AXIS_N) || (parser_state.sensor > VAC_SENSORS_N))
            return STATUS_CODE_MAX_VALUE_EXCEEDED;

        if (result == STATUS_OK) result = execute_command();
    } else {
        callbacks->command_executed(STATUS_OK); // Empty command returns OK immediately
    }

    return result;
}

/**
 * Non-Command Words: This initial parsing phase only checks for repeats of the remaining
 * legal g-code words and stores their value. Error-checking is performed later since some
 * words (I,J,K,L,P,R) have multiple connotations and/or depend on the issued commands.
 *
 * @return
 */
uint8_t TextControl::process_parameter() {
    auto line_state = &parser_state.line_state;
    uint8_t word_bit;
    switch(line_state->letter) {
        case 'Z':
            // Allow traveling only in negative direction
            if ((line_state->value < -settings->get_linear_max_travel()) || (line_state->value > 0))
                return STATUS_CODE_MAX_VALUE_EXCEEDED;
            word_bit = WORD_Z;
            parser_state.depth = line_state->value;
            break;
        case 'A': word_bit = WORD_A; parser_state.angle = line_state->value; break;
        case 'F': word_bit = WORD_F; parser_state.feed = line_state->value; break;
        case 'R': word_bit = WORD_R; line_state->light_color |= (((uint8_t) line_state->value & 0xff) << 8); break;
        case 'G': word_bit = WORD_G; line_state->light_color |= (((uint8_t) line_state->value & 0xff) << 16); break;
        case 'B': word_bit = WORD_B; line_state->light_color |= ((uint8_t) line_state->value & 0xff); break;
        default: return STATUS_CODE_UNSUPPORTED_COMMAND;
    }

    // Variable 'word_bit' is always assigned, if the non-command letter is valid.
    if ((line_state->word_bits & (1 << word_bit)) != 0) return STATUS_CODE_WORD_REPEATED; // [Word repeated]
    line_state->word_bits |= (1 << word_bit);

    // If we got light command with RGB arguments, then change the color
    if (line_state->command == COMMAND_LIGHT) {
        if (line_state->word_bits & ((1 << WORD_R) | (1 << WORD_G) | (1 << WORD_B))) {
            if (line_state->value > 255) return STATUS_CODE_MAX_VALUE_EXCEEDED;
            else parser_state.light_color = line_state->light_color;
        }
    }

    if ((line_state->command != COMMAND_NONE) && (line_state->command != COMMAND_LIGHT)) {
        // Require feed rate on move commands
        if ((line_state->word_bits & (1 << WORD_F)) == 0) return STATUS_CODE_UNDEFINED_FEED_RATE;

        // Check for parameters that are required by command
        if (line_state->command == COMMAND_ROTATE) {
            // Require angle on rotation command
            if ((line_state->word_bits & (1 << WORD_A)) == 0) return STATUS_CODE_INVALID_TARGET;
        } else {
            // Require depth on move or pick-place commands
            if ((line_state->word_bits & (1 << WORD_Z)) == 0) return STATUS_CODE_INVALID_TARGET;
        }
    }

    return STATUS_OK;
}

uint8_t TextControl::process_relay() {
    auto line_state = &parser_state.line_state;
    if (line_state->mantissa > 0) return STATUS_CODE_COMMAND_VALUE_NOT_INTEGER; // [No Mxx.x commands]
    switch(line_state->int_value) {
        case 0: case 1: case 2: case 3: // Relay ON commands
            parser_state.relay |= (1 << (line_state->int_value));
            break;

        case 10: case 11: case 12: case 13: // Relay OFF commands
            parser_state.relay &= ~(1 << (line_state->int_value - 10));
            break;

        default: return STATUS_CODE_INVALID_TARGET;

    }

    return STATUS_OK;
}

void TextControl::read_serial_input() {
    uint8_t ch;
    if ((ch = report->serial_read()) != SERIAL_NO_DATA) {
        if (ch == '\n') {
            uint8_t result;
            if (!line_too_long) {
                if (line[0] == '?') { report_state(); report->status_message(STATUS_OK); }
                else if (line[0] == 'H') { result = homing();  report->status_message(result); }
                else {
                    // Do not report execution until command is executed, but report errors immediately!
                    result = parse_serial_input();
                    if (result != STATUS_OK) report->status_message(result);
                }
            } else {
                result = STATUS_OVERFLOW;
                line_too_long = false;
            }
            ch_cnt = 0;
        } else if ((ch == '\r') || (ch == ' ') || (ch == '\b')) asm volatile("nop"); // Skip CR or space symbol, do nothing
        else if (ch == 0) ch_cnt = 0; // When comes 0 then reset the line
        else if (ch >= 'a' && ch <= 'z') line[ch_cnt++] = ch - 'a' + 'A'; // Lowercase to uppercase
        else if (ch_cnt >= LINE_MAX_LENGTH) line_too_long = true; // Otherwise, set 'too long flag' and that's it
        else line[ch_cnt++] = ch; // Add char to line until line is under 80 chars
        line[ch_cnt] = 0; // Set EOL
    }
}

uint8_t TextControl::execute_command() {
    if (!get_state()->is_state(STATE_IDLE)) return STATUS_BUSY;

    // Copy parser state values to FSM target state params
    get_state()->params.depth = parser_state.depth;
    get_state()->params.light_color = parser_state.light_color;
    get_state()->params.feed = parser_state.feed;
    get_state()->params.nozzle = parser_state.nozzle;
    get_state()->params.relays = parser_state.relay;
    get_state()->params.angle = parser_state.angle;

    switch (parser_state.line_state.command) {
        case COMMAND_PICK:
            get_state()->set_pick_place_state(STATE_PNP_CYCLE_NONE);
            get_state()->set_state(STATE_CYCLE_PICK);
            break;

        case COMMAND_PLACE:
            get_state()->set_pick_place_state(STATE_PNP_CYCLE_NONE);
            get_state()->set_state(STATE_CYCLE_PLACE);
            break;

        case COMMAND_ROTATE:
            get_state()->set_pick_place_state(STATE_PNP_CYCLE_NONE);
            get_state()->set_state(STATE_CYCLE_ROTATE);
            break;

        case COMMAND_MOVE:
            get_state()->set_pick_place_state(STATE_PNP_CYCLE_NONE);
            get_state()->set_state(STATE_CYCLE_MOVE);
            break;

        case COMMAND_LIGHT:
            get_state()->set_pick_place_state(STATE_PNP_CYCLE_NONE);
            get_state()->set_state(STATE_CYCLE_LIGHT);
            break;

        case COMMAND_RELAY:
            get_state()->set_pick_place_state(STATE_PNP_CYCLE_NONE);
            get_state()->set_state(STATE_CYCLE_RELAY);
            break;

        case COMMAND_READ_SENSOR:
#ifndef MODBUS
            report->print_float(vacuum->get_value(parser_state.sensor));
#endif
            callbacks->command_executed(STATUS_OK);
            break;

        default: break;
    }

    return STATUS_OK;
}

void TextControl::report_state() {
    report->print_state(vacuum,
                        state->end_stops,
                        state->relays,
                        state->light_color);
}
