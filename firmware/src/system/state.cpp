#include "system/state.h"

#include <cstring>
#include <cstdlib>

State::State() {
    this->state = STATE_IDLE;
    this->pick_place_state = 0;
    this->positions = (int32_t *) malloc(sizeof(int32_t) * AXIS_N);
}

void State::init() {
    this->state = STATE_IDLE;
    this->pick_place_state = 0;
    memset(this->positions, 0, sizeof(int32_t) * AXIS_N);
    set_end_motion();
}
