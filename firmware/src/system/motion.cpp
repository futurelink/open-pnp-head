#include "system/motion.h"
#include "system/parser.h"

#include <cmath>

Motion::Motion(Settings *settings, State *state) {
    this->settings = settings;
    this->state = state;
    this->current_block = nullptr;
}

motion_block_t *Motion::get_current_block() {
    return current_block;
}

void Motion::discard_current_block() {
    free(current_block);
    current_block = nullptr;
}

void Motion::move(uint8_t nozzle, float depth, float speed) {
    if (nozzle > ROTARY_AXIS_N - 1) return; // Invalid nozzle
    if (current_block == nullptr) {
        auto block = (motion_block_t *) malloc(sizeof(motion_block_t));
        auto motor_axis = ROTARY_AXIS_N + (uint8_t) floorf((float) nozzle / 2);
        auto position_steps = state->get_positions()[motor_axis];

        // - in all cases we need to return to 0 (-position) and then
        // - for even nozzles we need to go CCW i.e. negative
        // - for odd nozzles we need to go CW i.e. positive
#ifndef ROCKER_HEAD
        auto steps_per_mm = (float) settings->get_steps_per_mm(motor_axis);
        auto depth_steps = depth * steps_per_mm * ((nozzle % 2) ? 1.0f : -1.0f);
        auto target_steps = lroundf((float) depth_steps);
        auto delta = (float) (target_steps - position_steps) / steps_per_mm;
        auto delta_steps = (uint32_t) abs(target_steps - position_steps);
#else
        auto depth_signed = depth * ((nozzle % 2) ? 1.0f : -1.0f);
        auto position = settings->get_rocker_axis_position(position_steps);
        auto delta = depth_signed - position;
        auto delta_steps = lroundf(settings->get_rocker_axis_steps(position, depth_signed));
#endif

        if (delta_steps == 0) { // Nothing to add...
            free(block);
            return;
        }

        block->direction_bits = 0;
        block->units = fabsf(delta);
        block->acceleration = settings->get_acceleration(motor_axis);    // mm/s^2
        block->speed = min(speed, settings->get_speed(motor_axis));      // mm/s

        for (uint8_t idx = 0; idx < (uint8_t) AXIS_N; idx++) {
            block->steps[idx] = (idx == motor_axis) ? delta_steps : 0;
        }

        // Even nozzles motor direction CCW, odd nozzles motor direction CW
        if (delta < 0.0f) block->direction_bits |= (1 << motor_axis);

        block->step_event_count = block->steps[motor_axis];
        current_block = block;
    }
}

void Motion::rotate(uint8_t nozzle, float angle, float speed) {
    if (nozzle > ROTARY_AXIS_N - 1) return; // Invalid nozzle
    if (current_block == nullptr) {
        auto block = (motion_block_t *) malloc(sizeof(motion_block_t));
        auto position = state->get_positions()[nozzle]; // Position in steps
        auto target_steps = lroundf(angle * settings->get_steps_per_degree(nozzle));
        auto delta = (float) (target_steps - position) / settings->get_steps_per_degree(nozzle);
        auto delta_steps = (uint32_t) abs(target_steps - position);
        if (delta_steps == 0) { // Nothing to add...
            free(block);
            return;
        }

        block->direction_bits = 0;
        block->units = fabsf(delta);
        block->acceleration = settings->get_acceleration(nozzle);   // degrees/s^2
        block->speed = min(speed, settings->get_speed(nozzle));     // degrees/s
        for (uint8_t idx = 0; idx < (uint8_t) AXIS_N; idx++) {
            block->steps[idx] = (idx == nozzle) ? delta_steps : 0;
        }
        block->step_event_count = block->steps[nozzle];

        // Set direction bits: bit enabled means direction is negative
        if (delta < 0.0f) block->direction_bits |= (1 << nozzle);

        current_block = block;
    }
}

/**
 * Checks if nozzle is at the top position, and position is synchronized.
 * @param nozzle
 * @return
 */
bool Motion::check_nozzle_in_position(uint8_t nozzle) {
    if (current_block == nullptr) {
        auto motor_axis = ROTARY_AXIS_N + (uint8_t) floorf((float) nozzle / 2);
        auto position = state->get_positions()[motor_axis];
        return (position == 0);
    }
    return false;
}
