#include "system/control.h"
#include "system/macros.h"

#include <cmath>

Control::Control(Settings *settings, Report *report, Motion *motion, State *state, Callbacks *callbacks) {
    this->settings = settings;
    this->state = state;
    this->report = report;
    this->motion = motion;
    this->callbacks = callbacks;

    this->steppers = new Steppers(settings, state, callbacks);
    this->relay = new Relay();
    this->end_stops = new EndStops();
    this->vacuum = new Vacuum();
}

void Control::init() {
    steppers->init();
    relay->init();
    end_stops->init();
    vacuum->init();

    this->parser_state = {};
}

uint8_t Control::parse_line(char *line) {
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
        if (!read_float(line, &char_counter, &line_state->value)) return STATUS_BAD_NUMBER_FORMAT;

        // Get integer value for command code (e.g. M5, P10)
        line_state->int_value = (uint8_t) truncf(line_state->value);
        line_state->mantissa = (uint16_t) lroundf(100 * (line_state->value - (float) line_state->int_value));

        // Pick command P[nozzle] Z[depth], i.e. P0 Z-10, P1 Z-5, etc.
        // Place command the same with 'L' i.e. L0 Z-8, L1 Z-10, etc
        // Rotate command R[nozzle] A[degree] i.e. R1 A90
        // Valve relay command V[0-4]
        if (line_state->command) {
            result = process_parameter();
        } else {
            switch(parser_state.line_state.letter) {
                case 'R': line_state->command = COMMAND_ROTATE; parser_state.nozzle = line_state->int_value; break;
                case 'T': line_state->command = COMMAND_PICK; parser_state.nozzle = line_state->int_value; break;
                case 'P': line_state->command = COMMAND_PLACE; parser_state.nozzle = line_state->int_value; break;
                case 'M': line_state->command = COMMAND_MOVE; parser_state.nozzle = line_state->int_value; break;
                case 'L': line_state->command = COMMAND_LIGHT; break;
                case 'V': line_state->command = COMMAND_RELAY; result = process_relay(); break;
                default: result = STATUS_CODE_UNSUPPORTED_COMMAND;
            }
        }
    }

    // Command must have valid nozzle number
    if (line_state->command) if (parser_state.nozzle > ROTARY_AXIS_N) return STATUS_CODE_MAX_VALUE_EXCEEDED;

    if (result == STATUS_OK) result = callbacks->execute_command(&parser_state);

    return result;
}

/**
 * Non-Command Words: This initial parsing phase only checks for repeats of the remaining
 * legal g-code words and stores their value. Error-checking is performed later since some
 * words (I,J,K,L,P,R) have multiple connotations and/or depend on the issued commands.
 *
 * @return
 */
uint8_t Control::process_parameter() {
    auto line_state = &parser_state.line_state;
    uint8_t word_bit;
    switch(line_state->letter) {
        case 'Z':
            if (line_state->value > settings->get_linear_max_travel())
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

uint8_t Control::process_relay() {
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

void Control::execute_realtime() {
    // -----------------------------------------------------------------------------------------------------------------
    // Pick-Place cycle state machine
    // -----------------------------------------------------------------------------------------------------------------
    // Brief description ro PICK and PLACE cycles:
    // 1) STATE_CYCLE_MOVE_DOWN - wait for motion end (STATE_CYCLE_STOP bit)
    // 2) STATE_CYCLE_WAIT_PICK_UP or STATE_CYCLE_WAIT_RELEASE - depends on what is the action,
    //    waits for timeout or vacuum reaches value of locked nozzle
    // 3.1) STATE_CYCLE_ALARM - when component was not picked or placed
    // 3.1) STATE_CYCLE_MOVE_UP - when vacuum was ok, and waits for motion ends (STATE_CYCLE_STOP bit set)
    // 4) set global state to STATE_IDLE
    // -----------------------------------------------------------------------------------------------------------------
    // Brief description for ROTATE cycle:
    // 1) STATE_CYCLE_ROTATING - wait for motion end (STATE_CYCLE_STOP bit)
    // 2) set global state to STATE_IDLE
    // -----------------------------------------------------------------------------------------------------------------
    if (state->get_pick_place_state() & STATE_PNP_CYCLE_ALARM) {
        state->set_pick_place_state(0);
        state->set_state(STATE_ALARM);
    }

    else if (state->has_state(STATE_CYCLE_PICK | STATE_CYCLE_PLACE)) {
        if (!state->get_pick_place_state()) {
            // Pick-place cycle can only be started when nozzle is in neutral position
            if (motion->check_nozzle_in_position(parser_state.nozzle)) {
                // Start the cycle - move down to pick or place a component
                state->set_active_nozzle(parser_state.nozzle);
                motion->move(parser_state.nozzle, parser_state.depth, parser_state.feed);
                state->set_pick_place_state(STATE_PNP_CYCLE_MOVE_DOWN);
                steppers_start();
            } else {
                state->set_pick_place_state(0);
                state->set_state(STATE_ALARM);
            }
        } else if (state->get_pick_place_state() & STATE_PNP_CYCLE_MOVE_DOWN) {
            // If we reached the destination then move to pick wait phase
            if (state->has_state(STATE_CYCLE_STOP)) {
                if (state->has_state(STATE_CYCLE_PICK)) {
                    state->set_vac_wait_timer(PICK_WAIT_TIME);
                    state->set_pick_place_state(STATE_PNP_CYCLE_WAIT_VACUUM);
                    state->set_state(state->get_state() & ~STATE_CYCLE_STOP);
                } else if (state->has_state(STATE_CYCLE_PLACE)) {
                    state->set_vac_wait_timer(PLACE_WAIT_TIME);
                    state->set_pick_place_state(STATE_PNP_CYCLE_WAIT_VACUUM);
                    state->set_state(state->get_state() & ~STATE_CYCLE_STOP);
                } else {
                    state->set_pick_place_state(0);
                    state->set_state(STATE_ALARM);
                }
            }
        } else if (state->get_pick_place_state() & STATE_PNP_CYCLE_WAIT_VACUUM) {
            // Follow the vacuum measure and time and decide if we picked up a component.
            // If unsuccessful - then set to alarm state, otherwise go up
            if (state->is_vac_wait_timeout()) state->set_pick_place_state(STATE_PNP_CYCLE_ALARM);
            else if (vacuum->has_component(state->get_active_nozzle())) {
                motion->move(state->get_active_nozzle(), 0, parser_state.feed);
                state->set_pick_place_state(STATE_PNP_CYCLE_MOVE_UP);
                steppers_start();
            }
        } else if (state->get_pick_place_state() & STATE_PNP_CYCLE_MOVE_UP) {
            // If we reached the destination (technically - 0 at Z axis), then drop the cycle
            if (state->has_state(STATE_CYCLE_STOP)) {
                state->set_pick_place_state(0);
                state->set_state(STATE_IDLE);
            }
        }
    }

    else if (state->has_state(STATE_CYCLE_MOVE)) {
        if (!state->get_pick_place_state()) {
            motion->move(parser_state.nozzle, parser_state.depth,parser_state.feed);
            state->set_pick_place_state(STATE_PNP_CYCLE_MOVE);
            steppers_start();
        } else if (state->get_pick_place_state() & STATE_PNP_CYCLE_MOVE) {
            if (state->has_state(STATE_CYCLE_STOP)) {
                state->set_pick_place_state(0);
                state->set_state(STATE_IDLE);
            }
        }
    }

    else if (state->has_state(STATE_CYCLE_ROTATE)) {
        if (!state->get_pick_place_state()) {
            motion->rotate(parser_state.nozzle, parser_state.angle, parser_state.feed);
            state->set_pick_place_state(STATE_PNP_CYCLE_ROTATING);
            steppers_start();
        } else if (state->get_pick_place_state() & STATE_PNP_CYCLE_ROTATING) {
            if (state->has_state(STATE_CYCLE_STOP)) {
                state->set_pick_place_state(0);
                state->set_state(STATE_IDLE);
            }
        }
    }

    if (state->has_state(STATE_CYCLE_PICK | STATE_CYCLE_PLACE | STATE_CYCLE_ROTATE | STATE_CYCLE_MOVE | STATE_CYCLE_STOP)) {
        steppers->prepare_buffer();     // Prepare and fill stepper buffer
    }
}

/**
 * Extracts a floating point value from a string. The following code is based loosely on
 * the avr-libc strtod() function by Michael Stumpf and Dmitry Xmelkov and many freely
 * available conversion method examples, but has been highly optimized for Grbl. For known
 * CNC applications, the typical decimal value is expected to be in the range of E0 to E-4.
 * Scientific notation is officially not supported by g-code, and the 'E' character may
 * be a g-code word on some CNC systems. So, 'E' notation will not be recognized.
 * NOTE: Thanks to Radu-Eosif Mihailescu for identifying the issues with using strtod().
 */
bool Control::read_float(const char *line, uint8_t *char_counter, float *float_ptr) {
    auto ptr = (char*)line + *char_counter;
    uint8_t c = *ptr++; // Grab first character and increment pointer. No spaces assumed in line.

    // Capture initial positive/minus character
    int is_negative = false;
    if (c == '-') {
        is_negative = true;
        c = *ptr++;
    } else if (c == '+') {
        c = *ptr++;
    }

    // Extract number into fast integer. Track decimal in terms of exponent value.
    uint32_t int_val = 0;
    int8_t exp = 0;
    uint8_t n_digit = 0;
    int is_decimal = false;
    while (true) {
        c -= '0';
        if (c <= 9) {
            n_digit++;
            if (n_digit <= MAX_INT_DIGITS) {
                if (is_decimal) { exp--; }
                int_val = (((int_val << 2) + int_val) << 1) + c; // intval*10 + c
            } else {
                if (!(is_decimal)) exp++;  // Drop overflow digits
            }
        } else if (c == (('.'-'0') & 0xff)  &&  !is_decimal) {
            is_decimal = true;
        } else break;
        c = *ptr++;
    }

    // Return if no digits have been read.
    if (!n_digit) return false;

    // Convert integer into floating point.
    auto f_val = (float) int_val;

    // Apply decimal. Should perform no more than two floating point multiplications for the
    // expected range of E0 to E-4.
    if (f_val != 0) {
        while (exp <= -2) {
            f_val *= 0.01f;
            exp += 2;
        }
        if (exp < 0) f_val *= 0.1f;
        else if (exp > 0) {
            do {
                f_val *= 10.0f;
            } while (--exp > 0);
        }
    }

    *float_ptr = (is_negative) ? -f_val : f_val; // Assign floating point value with correct sign.
    *char_counter = ptr - line - 1;              // Set char_counter to next statement

    return true;
}

void Control::end_stops_interrupt() { end_stops->interrupt(); }

void Control::vacuum_set_value(uint8_t channel, uint16_t value) { vacuum->set_value(channel, value); }
void Control::vacuum_refresh() { vacuum->refresh(); }

void Control::steppers_pulse_start() { steppers->pulse_start(); }
void Control::steppers_pulse_end() { steppers->pulse_end(); }
void Control::steppers_start() {
    state->normal_motion();
    steppers->prepare_buffer();
    steppers->wake_up();
}

void Control::report_state() { report->print_state(vacuum, end_stops->get_state()); }

void Control::execute_relay(uint8_t st) {
    relay->set_state(st);
}
