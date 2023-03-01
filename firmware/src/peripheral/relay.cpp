#include "peripheral/relay.h"
#include "stm32/stm32_routines.h"

void Relay::init() {
    stm32_relay_init();
    stm32_set_relay_state(0x00);
}

void Relay::set_state(uint8_t state) {
    stm32_set_relay_state(state);
}
