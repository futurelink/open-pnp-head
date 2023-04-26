/*
  control.cpp - main control routines
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
#include "functions.h"

#include "system/control.h"
#include "system/macros.h"

#include "stm32/stm32_routines.h"

Control::Control(Settings *settings, Motion *motion, State *state, Callbacks *callbacks) {
    this->steppers = new Steppers(settings, state, callbacks);
    this->relay = new Relay();
    this->end_stops = new EndStops();
    this->vacuum = new Vacuum();

    this->settings = settings;
    this->state = state;
    this->motion = motion;
    this->callbacks = callbacks;
}

void Control::init() {
    steppers->init();
    relay->init();
    end_stops->init();
    vacuum->init();
}

/**
 * Disables steppers (turns off enable signal). This can be done only when
 * all linear axes are in upper position so that for rocker head springs are not
 * tensioned.
 */
void Control::check_and_disable_steppers() {
    if (end_stops->all_at_zero()) stm32_steppers_disable();
}

void Control::read_serial_input() {}

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
        state->set_pick_place_state(STATE_PNP_CYCLE_NONE);
        state->set_state(STATE_ALARM);
        check_and_disable_steppers();
        callbacks->command_executed(STATUS_CODE_INVALID_TARGET);
    }

    // Pick or place state
    else if (state->has_state(STATE_CYCLE_PICK | STATE_CYCLE_PLACE)) {
        // Initial point for pick-place cycle
        if (state->get_pick_place_state() == STATE_PNP_CYCLE_NONE) {
            // Pick-place cycle can only be started when nozzle is in neutral position and all end-stop
            // are not active (all heads are in 'upper' position)
            if (motion->check_nozzle_in_position(state->params.nozzle) && end_stops->all_at_zero()) {
                // Start the cycle - move down to pick or place a component
                state->set_active_nozzle(state->params.nozzle);
                motion->move(state->params.nozzle, state->params.depth, state->params.feed);
                state->set_pick_place_state(STATE_PNP_CYCLE_MOVE_DOWN);
                steppers_start();
            } else {
                state->set_pick_place_state(STATE_PNP_CYCLE_NONE);
                callbacks->command_executed(STATUS_CODE_INVALID_TARGET);
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
                    state->set_pick_place_state(STATE_PNP_CYCLE_NONE);
                    state->set_state(STATE_ALARM);
                    callbacks->command_executed(STATUS_CODE_INVALID_TARGET);
                }
            }
        } else if (state->get_pick_place_state() & STATE_PNP_CYCLE_WAIT_VACUUM) {
#ifdef VAC_SENSORS_N
            // Follow the vacuum measure and time and decide if we picked up a component.
            // If unsuccessful - then set to alarm state, otherwise go up
            if (state->is_vac_wait_timeout()) state->set_pick_place_state(STATE_PNP_CYCLE_ALARM);
            else
#endif
                if (vacuum->has_component(state->get_active_nozzle())) {
                motion->move(state->get_active_nozzle(), 0, state->params.feed);
                state->set_pick_place_state(STATE_PNP_CYCLE_MOVE_UP);
                steppers_start();
            }
        } else if (state->get_pick_place_state() & STATE_PNP_CYCLE_MOVE_UP && state->has_state(STATE_CYCLE_STOP)) {
            // If we reached the destination (technically - 0 at Z axis), then drop the cycle.
            // Check if we have all end-stops set to 0, if not - then this is an error.
            if (end_stops->all_at_zero()) {
                state->set_pick_place_state(STATE_PNP_CYCLE_NONE);
                state->set_state(STATE_IDLE);
                check_and_disable_steppers();
                callbacks->command_executed(STATUS_OK);
            } else {
                state->set_pick_place_state(STATE_PNP_CYCLE_ALARM);
                callbacks->command_executed(STATUS_CODE_INVALID_TARGET);
            }
        }
    }

    // Movement state
    else if (state->has_state(STATE_CYCLE_MOVE)) {
        if (!state->get_pick_place_state()) {
            motion->move(state->params.nozzle, state->params.depth, state->params.feed);
            state->set_pick_place_state(STATE_PNP_CYCLE_MOVE);
            steppers_start();
        } else if ((state->get_pick_place_state() & STATE_PNP_CYCLE_MOVE) && state->has_state(STATE_CYCLE_STOP)) {
            // Check if we wanted to move to 0 position, which means end-stops should have gone 0,
            // if any end-stop is not 0, then this is an error.
            if ((state->params.depth == 0) && !end_stops->all_at_zero()) {
                state->set_pick_place_state(STATE_PNP_CYCLE_ALARM);
                callbacks->command_executed(STATUS_CODE_INVALID_TARGET);
            } else {
                state->set_pick_place_state(STATE_PNP_CYCLE_NONE);
                state->set_state(STATE_IDLE);
                check_and_disable_steppers();
                callbacks->command_executed(STATUS_OK);
            }
        }
    }

    // Rotation state
    else if (state->has_state(STATE_CYCLE_ROTATE)) {
        if (!state->get_pick_place_state()) {
            motion->rotate(state->params.nozzle, state->params.angle, state->params.feed);
            state->set_pick_place_state(STATE_PNP_CYCLE_ROTATING);
            steppers_start();
        } else if (state->get_pick_place_state() & STATE_PNP_CYCLE_ROTATING) {
            if (state->has_state(STATE_CYCLE_STOP)) {
                state->set_pick_place_state(STATE_PNP_CYCLE_NONE);
                state->set_state(STATE_IDLE);
                check_and_disable_steppers();
                callbacks->command_executed(STATUS_OK);
            }
        }
    }

    // Light change state
    if (state->has_state(STATE_CYCLE_LIGHT)) {
        state->light_color = state->params.light_color;
        state->set_state(STATE_IDLE);
        callbacks->command_executed(STATUS_OK);
    }

    // Relay change state
    if (state->has_state(STATE_CYCLE_RELAY)) {
        state->relays = state->params.relays;
        state->set_state(STATE_IDLE);
        callbacks->command_executed(STATUS_OK);
    }

    if (state->has_state(STATE_CYCLE_PICK | STATE_CYCLE_PLACE | STATE_CYCLE_ROTATE | STATE_CYCLE_MOVE | STATE_CYCLE_STOP)) {
        steppers->prepare_buffer();     // Prepare and fill stepper buffer
    }

    sync(); // Synchronize internal state with hardware
}

/**
 * TODO Not implemented yet
 * Runs homing cycle for belt-driven linear axes.
 * Not applicable for rocker-heads.
 * @return
 */
uint8_t Control::homing() {
#ifdef ROCKER_HEAD
    return STATUS_CODE_UNSUPPORTED_COMMAND;
#else
    return STATUS_OK;
#endif
}

void Control::end_stops_interrupt() { end_stops->interrupt(); }

void Control::vacuum_set_value(uint8_t channel, uint16_t value) {
    vacuum->set_value(channel, value);
    state->set_vacuum(channel, vacuum->get_value(channel));
}
void Control::vacuum_refresh() { vacuum->refresh(); }

void Control::steppers_pulse_start() { steppers->pulse_start(); }
void Control::steppers_pulse_end() { steppers->pulse_end(); }
void Control::steppers_start() {
    state->normal_motion();
    steppers->prepare_buffer();
    steppers->wake_up();
}

void Control::sync() {
    state->end_stops = end_stops->get_state();
    relay->set_state(state->relays);
    stm32_light_set_color(state->light_color);
}
