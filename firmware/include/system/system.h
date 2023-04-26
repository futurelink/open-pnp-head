/*
  system.h
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

#include "config.h"
#include "state.h"
#include "serial.h"
#include "stepper.h"
#include "settings.h"
#include "motion.h"

#ifdef MODBUS
#include "modbus_impl.h"
#include "modbus_control.h"
#else
#include "report.h"
#include "text_control.h"
#endif

#include "peripheral/vacuum.h"

#include "stm32/stm32_routines.h"

#ifndef SYSTEM_H
#define SYSTEM_H

class System {
private:
    static State    *state;
    static Settings *settings;
    static Motion   *motion;

#ifdef MODBUS
    static ModBus   *modbus;
    static Control  *control;
#else
    static Serial   *serial;
    static Report   *report;
    static Control  *control;
#endif

    static Callbacks callbacks;

    /***********************************************************************
     * Callback methods implementations
     ***********************************************************************/

    /**********************************************************************/

    System() = default; // Not an instantiatable class

    [[noreturn]] static void main_loop();

    static motion_block_t   *get_current_block();
    static void             discard_current_block();
    static void             command_executed(uint8_t status);

public:
    static void run();

    static void heartbeat();

    static void adc_read(uint8_t channel, uint16_t value);
    static void steppers_pulse_start();
    static void steppers_pulse_end();

    static void external_interrupt_limit();

    static void silence_timer_fired();
    static void usart_transmit();
    static void usart_receive();
};


#endif //SYSTEM_H
