#include "peripheral/endstops.h"
#include "stm32/stm32_routines.h"

void EndStops::init() {
    stm32_limits_init();
}

void EndStops::interrupt() {

}

uint8_t EndStops::get_state() {
    auto pin_state = stm32_limits_get_state();
    return ((pin_state & (1 << LIMIT_1_BIT)) ? 0b0001 : 0) |
            ((pin_state & (1 << LIMIT_2_BIT)) ? 0b0010 : 0) |
            ((pin_state & (1 << LIMIT_3_BIT)) ? 0b0100 : 0) |
            ((pin_state & (1 << LIMIT_4_BIT)) ? 0b1000 : 0);
}

/**
 * Return true if all end-stops are at initial position.
 *
 * @return
 */
bool EndStops::all_at_zero() {
    return (get_state() == 0);
}
