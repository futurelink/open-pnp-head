/*
  stepper.cpp - step motor control routines
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

#include <cmath>
#include <cstring>

#include "config.h"
#include "system/stepper.h"
#include "system/macros.h"
#include "stm32/stm32_routines.h"

Steppers::Steppers(Settings *settings, State *state, Callbacks *callbacks) {
    this->settings = settings;
    this->state = state;
    this->callbacks = callbacks;
    this->step_port_invert_mask = 0xffff;
    this->dir_port_invert_mask = 0;

    // Initialize structs with zeroes
    memset(&profile, 0, sizeof(profile_prep_t));
    memset(&st, 0, sizeof(stepper_t));

    this->planned_block = nullptr;
    this->segment_buffer_tail = 0;
    this->segment_buffer_head = 0;
    this->segment_next_head = 1;
    this->busy = false;

    this->segment_dt = 0;
    this->units_remaining = 0;
}

void Steppers::init() {
    stm32_stepper_init();
    go_idle();
    stm32_steppers_set(dir_port_invert_mask, step_port_invert_mask);
}

void Steppers::pulse_start() {
    if (!stm32_steppers_pulse_start(busy, st.dir_out_bits, st.step_out_bits)) return;

    busy = true;

    // If there is no step segment, attempt to pop one from the stepper buffer
    if (st.exec_segment == nullptr) {
        if (segment_buffer_head != segment_buffer_tail) { // Anything in the buffer? If so, load and initialize next step segment.
            st.exec_segment = &segment_buffer[segment_buffer_tail]; // Initialize new step segment and load number of steps to execute

            // Initialize step segment timing per step and load number of steps to execute.
            stm32_steppers_set_timer(st.exec_segment->cycles_per_tick);
            st.step_count = st.exec_segment->n_step;

            // If the new segment starts a new planner block, initialize stepper variables and counters.
            // NOTE: When the segment data index changes, this indicates a new planner block.
            if (st.exec_block_index != st.exec_segment->st_block_index) {
                st.exec_block_index = st.exec_segment->st_block_index;
                st.exec_block = &st_block_buffer[st.exec_block_index];

                // Initialize Bresenham line and distance counters
                for (uint32_t &counter : st.counters) counter = st.exec_block->step_event_count >> 1;
            }

            st.dir_out_bits = st.exec_block->dir_port_bits ^ dir_port_invert_mask;

            // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level
            for (uint8_t idx = 0; idx < (uint8_t) AXIS_N; idx++) {
                st.steps[idx] = st.exec_block->steps[idx] >> st.exec_segment->amass_level;
            }
        } else {
            go_idle(); // Segment buffer empty. Shutdown.
            state->set_state(state->get_state() | STATE_CYCLE_STOP);  // Flag for cycle end, keep cycle flag
            return;
        }
    }

    st.step_out_bits = 0; // Reset step out bits.

    // Execute step displacement profile by Bresenham line algorithm
    // -------------------------------------------------------------
    for (uint8_t idx = 0; idx < (uint8_t) AXIS_N; idx++) {
        st.counters[idx] += st.steps[idx];
        if (st.counters[idx] > st.exec_block->step_event_count) {
            st.step_out_bits |= step_pin_mask[idx];
            st.counters[idx] -= st.exec_block->step_event_count;
            if (st.dir_out_bits & direction_pin_mask[idx])
                state->decrement_position(idx);
            else
                state->increment_position(idx);
        }
    }

    st.step_count--; // Decrement step events count
    if (st.step_count == 0) {
        st.exec_segment = nullptr; // Segment is complete. Discard current segment and advance segment indexing.

        uint8_t segment_tail_next = segment_buffer_tail + 1;
        segment_buffer_tail = (segment_tail_next == SEGMENT_BUFFER_SIZE) ? 0 : segment_tail_next;
    }

    st.step_out_bits ^= step_port_invert_mask;  // Apply step port invert mask
    busy = false;
}

void Steppers::pulse_end() const {
    stm32_steppers_pulse_end(step_port_invert_mask);
}

/**
 * Stepper state initialization
 */
void Steppers::wake_up() {
    stm32_steppers_enable();

    // Initialize stepper output bits to ensure first ISR call does not step.
    st.step_out_bits = step_port_invert_mask;

    // Set step pulse time. Ad-hoc computation from oscilloscope. Uses two's complement.
    st.step_pulse_time = STEP_PULSE_MS * TICKS_PER_MICROSECOND;

    stm32_steppers_wake_up(st.step_pulse_time, st.exec_segment->cycles_per_tick, 0);
}

/**
 * Stepper shutdown
 */
void Steppers::go_idle() {
    stm32_steppers_go_idle();

    busy = false;
}

/**
 * Called by realtime status reporting to fetch the current speed being executed. This value
 * however is not exactly the current speed, but the speed computed in the last step segment
 * in the segment buffer. It will always be behind by up to the number of segment blocks (-1)
 * divided by the ACCELERATION TICKS PER SECOND in seconds.
 */
float Steppers::get_realtime_rate() const {
    return state->has_state(STATE_CYCLE_PICK | STATE_CYCLE_PLACE | STATE_CYCLE_ROTATE) ?
        profile.current_speed : 0.0f;
}

/**
 * Prepares buffer for stepper execution.
 * This method should be called as often as possible. Generally - from the main loop.
 */
void Steppers::prepare_buffer() {
    if (state->is_end_motion()) return;
    while (segment_buffer_tail != segment_next_head) {           // Check if we need to fill the buffer.
        if (!prepare_block()) return;                            // Bail out if there's no block to execute
        prepare_profile();
        prepare_segment(); // Initialize new segment
        if (check_end_motion()) {
            state->set_end_motion();
            return;
        }
    }
}

/**
 * Prepares speed profile structure for planned movement.
 *
 * @return
 */
bool Steppers::prepare_block() {
    if (planned_block != nullptr) return true; // Have block still processing

    planned_block = callbacks->get_current_block();
    if (planned_block == nullptr) return false; // No new block, bail out.

    // Load the Bresenham stepping data for the block.
    profile.st_block_index = next_block_index(profile.st_block_index);

    // Prepare and copy Bresenham algorithm segment data from the new planner block, so that
    // when the segment buffer completes the planner block, it may be discarded when the
    // segment buffer finishes the prepped block, but the stepper ISR is still executing it.
    st_prep_block = &st_block_buffer[profile.st_block_index];

    // Prepare block execution data
    st_prep_block->dir_port_bits = axis_to_port_direction_bits(planned_block->direction_bits);
    st_prep_block->step_event_count = planned_block->step_event_count << MAX_AMASS_LEVEL;
    for (uint8_t idx = 0; idx < (uint8_t) AXIS_N; idx++)
        st_prep_block->steps[idx] = planned_block->steps[idx] << MAX_AMASS_LEVEL;

    // Initialize segment buffer data for generating the segments.
    profile.steps_remaining = (float) planned_block->step_event_count;
    profile.steps_per_unit = profile.steps_remaining / planned_block->units;
    profile.req_units_increment = REQ_MM_INCREMENT_SCALAR / profile.steps_per_unit;
    profile.dt_remainder = 0.0f;    // Reset for new segment block
    profile.current_speed = 0.0f;   // Initial speed is 0
    profile.units_complete = 0.0f;  // Default velocity profile complete at 0.0mm from end of block.

    // Compute or recompute velocity profile parameters of the prepped planner block
    profile.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp
    profile.accelerate_until = planned_block->units;
    profile.exit_speed = 0.0f; // Exit speed is 0, as we don't use planned motions with junctions

    auto inv_2_accel = 0.5f / planned_block->acceleration;
    auto entry_speed_sqr = 0.0f; // Entry speed is 0, as we don't use planned motions with junctions
    auto exit_speed_sqr = profile.exit_speed * profile.exit_speed;
    auto nominal_speed = planned_block->speed;
    auto nominal_speed_sqr = nominal_speed * nominal_speed;
    auto intersect_distance = 0.5f * (planned_block->units + inv_2_accel * (entry_speed_sqr - exit_speed_sqr));

    // Trapezoid or triangle profile
    // -----------------------------
    if (intersect_distance > 0.0f) {
        if (intersect_distance < planned_block->units) {
            profile.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
            if (profile.decelerate_after < intersect_distance) { // Trapezoid type
                profile.maximum_speed = nominal_speed;
                if (entry_speed_sqr == nominal_speed_sqr)
                    profile.ramp_type = RAMP_CRUISE;
                else
                    profile.accelerate_until -= inv_2_accel * (nominal_speed_sqr - entry_speed_sqr);
            } else { // Triangle type
                profile.accelerate_until = intersect_distance;
                profile.decelerate_after = intersect_distance;
                profile.maximum_speed = sqrtf(2.0f * planned_block->acceleration * intersect_distance + exit_speed_sqr);
            }
        } else { // Deceleration-only type
            profile.ramp_type = RAMP_DECELERATION;
        }
    } else { // Acceleration-only type
        profile.accelerate_until = 0.0f;
        profile.maximum_speed = profile.exit_speed;
    }

    return true;
}

/**
 * Re-calculates speed profile to determine when the axis should go cruise motion or decelerate.
 */
void Steppers::prepare_profile() {
    float dt_max = DT_SEGMENT;  // Maximum segment time
    float time_var = dt_max;    // Time worker variable
    float mm_var;               // mm-Distance worker variable
    float speed_var;            // Speed worker variable

    segment_dt = 0.0f;                      // Initialize segment time
    units_remaining = planned_block->units; // New segment distance from end of block.
    auto minimum_units = units_remaining - profile.req_units_increment;     // Guarantee at least one step.
    if (minimum_units < 0.0f) minimum_units = 0.0f;

    do {
        switch (profile.ramp_type) {
            case RAMP_ACCEL: // NOTE: Acceleration ramp only computes during first do-while loop.
                speed_var = planned_block->acceleration * time_var;
                units_remaining -= time_var * (profile.current_speed + 0.5f * speed_var);
                if (units_remaining < profile.accelerate_until) { // End of acceleration ramp.
                    // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
                    units_remaining = profile.accelerate_until; // NOTE: 0.0 at EOB
                    time_var = 2.0f * (planned_block->units - units_remaining) / (profile.current_speed + profile.maximum_speed);
                    if (units_remaining == profile.decelerate_after) profile.ramp_type = RAMP_DECELERATION;
                    else profile.ramp_type = RAMP_CRUISE;
                    profile.current_speed = profile.maximum_speed;
                } else { // Acceleration only.
                    profile.current_speed += speed_var;
                }
                break;
            case RAMP_CRUISE:
                // NOTE: mm_var used to retain the last units_remaining for incomplete segment time_var calculations.
                // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To
                //   prevent this, simply enforce a minimum speed threshold in the planner.
                mm_var = units_remaining - profile.maximum_speed * time_var;
                if (mm_var < profile.decelerate_after) { // End of cruise.
                    // Cruise-deceleration junction or end of block.
                    time_var = (units_remaining - profile.decelerate_after) / profile.maximum_speed;
                    units_remaining = profile.decelerate_after; // NOTE: 0.0 at EOB
                    profile.ramp_type = RAMP_DECELERATION;
                } else { // Cruising only.
                    units_remaining = mm_var;
                }
                break;
            default:
                // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
                speed_var = planned_block->acceleration * time_var; // Used as delta speed (mm/min)
                if (profile.current_speed > speed_var) { // Check if at or below zero speed.
                    // Compute distance from end of segment to end of block.
                    mm_var = units_remaining - time_var * (profile.current_speed - 0.5f * speed_var); // (mm)
                    if (mm_var > profile.units_complete) { // Typical case. In deceleration ramp.
                        units_remaining = mm_var;
                        profile.current_speed -= speed_var;
                        break; // Segment complete. Exit switch-case statement. Continue do-while loop.
                    }
                }
                // Otherwise, at end of block or end of forced-deceleration.
                time_var = 2.0f * (units_remaining - profile.units_complete) / (profile.current_speed + profile.exit_speed);
                units_remaining = profile.units_complete;
                profile.current_speed = profile.exit_speed;
        }
        segment_dt += time_var; // Add computed ramp time to total segment time.
        if (segment_dt < dt_max) time_var = dt_max - segment_dt; // **Incomplete** At ramp junction.
        else {
            if (units_remaining > minimum_units) {
                // Check for very slow segments with zero steps.
                // Increase segment time to ensure at least one step in segment. Override and loop
                // through distance calculations until minimum_units or units_complete.
                dt_max += DT_SEGMENT;
                time_var = dt_max - segment_dt;
            } else {
                break; // **Complete** Exit loop. Segment execution time maxed.
            }
        }
    } while (units_remaining > profile.units_complete); // **Complete** Exit loop. Profile complete.
}

bool Steppers::prepare_segment() {
    float step_dist_remaining = profile.steps_per_unit * units_remaining;  // Convert units_remaining to steps
    float n_steps_remaining = ceilf(step_dist_remaining);               // Round-up current steps remaining
    float last_n_steps_remaining = ceilf(profile.steps_remaining);         // Round-up last steps remaining

    auto prep_segment = &segment_buffer[segment_buffer_head];
    prep_segment->n_step = (uint16_t)(last_n_steps_remaining - n_steps_remaining); // Compute number of steps to execute
    prep_segment->st_block_index = profile.st_block_index;       // Set new segment to point to the current segment data block

    // Compute segment step rate. Since steps are integers and mm distances traveled are not,
    // the end of every segment can have a partial step of varying magnitudes that are not
    // executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
    // compensate, we track the time to execute the previous segment's partial step and simply
    // apply it with the partial step distance to the current segment, so that it minutely
    // adjusts the entire segment rate to keep step output exact. These rate adjustments are
    // typically very small and do not adversely affect performance, but ensures that it
    // outputs the exact acceleration and velocity profiles as computed by the planner.
    segment_dt += profile.dt_remainder; // Apply previous segment partial step execute time
    float inv_rate = segment_dt / (last_n_steps_remaining - step_dist_remaining); // Compute adjusted step rate inverse

    // Compute CPU cycles per step for the prepped segment.
    auto cycles = (uint32_t) ceilf(F_CPU * inv_rate); // (cycles/step)
    apply_amass(prep_segment, cycles);

    // Segment complete! Increment segment buffer indices, so stepper ISR can immediately execute it.
    segment_buffer_head = segment_next_head;
    if (++segment_next_head == SEGMENT_BUFFER_SIZE) segment_next_head = 0;

    // Update the appropriate planner and segment data.
    planned_block->units = units_remaining;
    profile.steps_remaining = n_steps_remaining;
    profile.dt_remainder = (n_steps_remaining - step_dist_remaining) * inv_rate;

    return false;
}

/**
* Checks for exit conditions and flag to load next planner block.
*/
bool Steppers::check_end_motion() {
    if (units_remaining == profile.units_complete) {
        // End of planner block or forced-termination. No more distance to be executed
        if (units_remaining > 0.0f) return true; // Bail!
        else { // End of planner block
            planned_block = nullptr; // Set pointer to indicate check and load next planner block
            callbacks->discard_current_block();
            return true;
        }
    }
    return false;
}

void Steppers::apply_amass(segment_t *segment, uint32_t cycles) {
    if (cycles < AMASS_LEVEL1) segment->amass_level = 0;
    else {
        if (cycles < AMASS_LEVEL2) segment->amass_level = 1;
        else if (cycles < AMASS_LEVEL3) segment->amass_level = 2;
        else segment->amass_level = 3;
        cycles >>= segment->amass_level;
        segment->n_step <<= segment->amass_level;
    }
    if (cycles < (1UL << 16)) segment->cycles_per_tick = cycles; // < 65536 (4.1ms @ 16MHz)
    else segment->cycles_per_tick = 0xffff; // Just set the slowest speed possible.

}

uint16_t Steppers::axis_to_port_direction_bits(uint16_t axis_bits) const {
    uint16_t bits = 0;
    for (uint8_t idx = 0; idx < (uint8_t) AXIS_N; idx++) if (axis_bits & (1 << idx)) bits |= direction_pin_mask[idx];
    return bits;
}

uint8_t Steppers::next_block_index(uint8_t block_index) {
    return (++block_index == (SEGMENT_BUFFER_SIZE - 1)) ? 0 : block_index;
}
