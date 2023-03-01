#include <cstdint>

#include "config.h"
#include "state.h"
#include "serial.h"
#include "stepper.h"
#include "report.h"
#include "settings.h"
#include "control.h"
#include "motion.h"

#include "peripheral/relay.h"
#include "peripheral/endstops.h"
#include "peripheral/vacuum.h"

#include "stm32/stm32_routines.h"

#ifndef SYSTEM_H
#define SYSTEM_H

class System {
private:
    static State    *state;
    static Settings *settings;
    static Serial   *serial;
    static Report   *report;
    static Control  *control;
    static Motion   *motion;

    static Callbacks callbacks;

    /***********************************************************************
     * Callback methods implementations
     ***********************************************************************/

    /**********************************************************************/

    System() = default; // Not an instantiatable class

    [[noreturn]] static void main_loop();

    static motion_block_t   *get_current_block();
    static void             discard_current_block();
    static uint8_t          execute_command(parser_state_t *parser_state);

public:
    static void run();

    static void heartbeat();

    static void adc_read(uint8_t channel, uint16_t value);
    static void steppers_pulse_start();
    static void steppers_pulse_end();

    static void external_interrupt_limit();

    static void usart_transmit();
    static void usart_receive();
};


#endif //SYSTEM_H
