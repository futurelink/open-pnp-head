/*
  system.cpp - main system class, fully static
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
        .command_executed       = command_executed
};

Settings       *System::settings     = new Settings();
State          *System::state        = new State();
Motion         *System::motion       = new Motion(settings, state);
#ifdef MODBUS
ModBus         *System::modbus       = new ModBusSlave(0x55, settings, state);
Control        *System::control      = new ModBusControl(settings, modbus, motion, state, &callbacks);
#else
Serial         *System::serial       = new Serial();
Report         *System::report       = new Report(settings, serial, state);
Control        *System::control      = new TextControl(settings, report, motion, state, &callbacks);
#endif

void System::run() {
    stm32_init();
    stm32_eeprom_init();
    stm32_system_init();
    stm32_light_init();

#ifndef MODBUS
    serial->init();
#endif

    state->init();
    control->init();
    settings->load();

#ifndef MODBUS
    for (long i = 0; i < 1000; i++) asm volatile("nop"); // Delay
    serial->print_string(GREETING_STRING);
#endif

    main_loop(); // Run main loop until reset signal is caught
}

/**
 * PRIMARY LOOP
 */
[[noreturn]] void System::main_loop() {
    while (true)  {
        control->read_serial_input();
        control->execute_realtime();
    }
}

motion_block_t *System::get_current_block() { return motion->get_current_block(); }
void System::discard_current_block() { motion->discard_current_block(); }
void System::command_executed(uint8_t status) {
#ifndef MODBUS
    report->status_message(status);
#endif
}

// ------------------------------------
//

/**
 * Heartbeat function is executed in SysTick timer, so it must fit in 1ms.
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

void System::silence_timer_fired() {
    modbus->silence();
}

void System::usart_transmit() {
#ifdef MODBUS
    modbus->transmit();
#else
    serial->transmit();
#endif
}

void System::usart_receive() {
#ifdef MODBUS
    modbus->receive();
#else
    serial->receive();
#endif
}
