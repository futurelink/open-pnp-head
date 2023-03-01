/*
  system.cpp - Main system class, fully static
  Part of pnp-head

  Copyright (c) 2022-2023 Denis Pavlov

  pnp-head is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  pnp-head is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with pnp-head. If not, see <http://www.gnu.org/licenses/>.
*/

#include "system/system.h"

Callbacks System::callbacks = {
        .get_current_block      = get_current_block,
        .discard_current_block  = discard_current_block,
        .execute_command        = execute_command
};

Settings       *System::settings     = new Settings();
Serial         *System::serial       = new Serial();
State          *System::state        = new State();
Motion         *System::motion       = new Motion(settings, state);
Report         *System::report       = new Report(settings, serial, state);
Control        *System::control      = new Control(settings, report, motion, state, &callbacks);

void System::run() {
    stm32_init();
    stm32_eeprom_init();
    stm32_system_init();

    state->init();
    serial->init();
    control->init();

    serial->print_string(GREETING_STRING);

    main_loop(); // Run main loop until reset signal is caught
}

/**
 * PRIMARY LOOP
 */
[[noreturn]] void System::main_loop() {
    char line[LINE_MAX_LENGTH];
    uint8_t cnt = 0;
    uint8_t ch;
    bool line_too_long = false;
    while (true)  {
        if ((ch = serial->read()) != SERIAL_NO_DATA) {
            if (ch == '\n') {
                uint8_t result;
                if (!line_too_long) {
                    if (line[0] == '?') { result = STATUS_OK; control->report_state(); }
                    else result = control->parse_line(line);
                } else {
                    result = STATUS_OVERFLOW;
                    line_too_long = false;
                }

                report->status_message(result);
                cnt = 0;
            } else if ((ch == '\r') || (ch == ' ') || (ch == '\b')) asm volatile("nop"); // Skip CR or space symbol, do nothing
            else if (ch == 0) cnt = 0; // When comes 0 then reset the line
            else if (ch >= 'a' && ch <= 'z') line[cnt++] = ch - 'a' + 'A'; // Lowercase to uppercase
            else if (cnt >= LINE_MAX_LENGTH) line_too_long = true; // Otherwise, set 'too long flag' and that's it
            else line[cnt++] = ch; // Add char to line until line is under 80 chars
            line[cnt] = 0; // Set EOL
        }

        control->execute_realtime();
    }
}

motion_block_t *System::get_current_block() { return motion->get_current_block(); }
void System::discard_current_block() { motion->discard_current_block(); }

// ------------------------------------
//

/**
 * Heartbeat function is executed in Systick timer, so it must fit in 1ms.
 */
void System::heartbeat() {
    control->vacuum_refresh();
    control->vacuum_timer_decrement();
}

void System::adc_read(uint8_t channel, uint16_t value) { control->vacuum_set_value(channel, value); }

/**
 * Stepper interface methods
 */
void System::steppers_pulse_start() { control->steppers_pulse_start(); }
void System::steppers_pulse_end() { control->steppers_pulse_end(); }

/**
 * External interrupts handlers
 */
void System::external_interrupt_limit() { control->end_stops_interrupt(); }

/**
 * Communication callbacks
 */
void System::usart_transmit() { serial->transmit(); }
void System::usart_receive() { serial->receive(); }

uint8_t System::execute_command(parser_state_t *parser_state) {
    if (!control->get_state()->is_state(STATE_IDLE))
        return STATUS_BUSY;

    control->execute_relay(parser_state->relay);

    switch (parser_state->line_state.command) {
        case COMMAND_PICK:
            control->get_state()->set_pick_place_state(0);
            control->get_state()->set_state(STATE_CYCLE_PICK);
            break;

        case COMMAND_PLACE:
            control->get_state()->set_pick_place_state(0);
            control->get_state()->set_state(STATE_CYCLE_PLACE);
            break;

        case COMMAND_ROTATE:
            control->get_state()->set_pick_place_state(0);
            control->get_state()->set_state(STATE_CYCLE_ROTATE);
            break;

        default: break;
    }

    return STATUS_OK;
}
