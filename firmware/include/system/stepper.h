/*
  stepper.h
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

#ifndef STEPPER_H
#define STEPPER_H

#include "settings.h"
#include "State.h"
#include "cpu_map.h"
#include "callbacks.h"

#define SEGMENT_BUFFER_SIZE 32

// AMASS was derived from GRBL's algorithm. If you want to learn more about
// how it works, please refer to GRBL documentation / source code.
// ------------------------------------------------------------------------
#define MAX_AMASS_LEVEL 3
// AMASS_LEVEL0: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
#define AMASS_LEVEL1 (F_CPU / 8000) // Over-drives ISR (x2). Defined as F_CPU/(Cutoff frequency in Hz)
#define AMASS_LEVEL2 (F_CPU / 4000) // Over-drives ISR (x4)
#define AMASS_LEVEL3 (F_CPU / 2000) // Over-drives ISR (x8)

enum RampType { RAMP_ACCEL, RAMP_CRUISE, RAMP_DECELERATION };

typedef struct {
    uint16_t    n_step;           // Number of step events to be executed for this segment
    uint16_t    cycles_per_tick;  // Step distance traveled per ISR tick, aka step rate.
    uint8_t     st_block_index;   // Stepper block data index. Uses this information to execute this segment.
    uint8_t     amass_level;      // Indicates AMASS level for the ISR to execute this segment
} segment_t;

typedef struct {
    uint32_t    steps[AXIS_N];
    uint32_t    step_event_count;
    uint16_t    dir_port_bits;
} st_block_t;

typedef struct {
    // Used by the bresenham line algorithm
    uint32_t    counters[AXIS_N]; // Counter variables for the bresenham line tracer
    uint32_t    steps[AXIS_N];

    uint8_t     execute_step;     // Flags step execution for each interrupt.
    uint8_t     step_pulse_time;  // Step pulse reset time after step rise

    uint16_t   step_out_bits;     // The next stepping-bits to be output
    uint16_t   dir_out_bits;

    uint16_t    step_count;        // Steps remaining in line segment motion
    uint8_t     exec_block_index;  // Tracks the current st_block index. Change indicates new block.
    st_block_t  *exec_block;       // Pointer to the block data for the segment being executed
    segment_t   *exec_segment;     // Pointer to the segment being executed
} stepper_t;

// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
    uint8_t     st_block_index;     // Index of stepper common data block being prepped
    uint8_t     recalculate_flag;

    float       dt_remainder;
    float       steps_remaining;
    float       steps_per_unit;
    float       req_units_increment;

    RampType    ramp_type;          // Current segment ramp state
    float       units_complete;        // End of velocity profile from end of current planner block in (mm).
    float       current_speed;      // Current speed at the end of the segment buffer (mm/min)
    float       maximum_speed;      // Maximum speed of executing block. Not always nominal speed. (mm/min)
    float       exit_speed;         // Exit speed of executing block (mm/min)
    float       accelerate_until;   // Acceleration ramp end measured from end of block (mm)
    float       decelerate_after;   // Deceleration ramp start measured from end of block (mm)
} profile_prep_t;

class Steppers {
private:
    const uint16_t step_pin_mask[AXIS_N] = {1 << ROTARY_1_STEP_BIT, 1 << ROTARY_2_STEP_BIT,
                                            1 << ROTARY_3_STEP_BIT, 1 << ROTARY_4_STEP_BIT,
                                            1 << LINEAR_1_STEP_BIT, 1 << LINEAR_2_STEP_BIT};
    const uint16_t direction_pin_mask[AXIS_N] = {1 << ROTARY_1_DIR_BIT, 1 << ROTARY_2_DIR_BIT,
                                                 1 << ROTARY_3_DIR_BIT, 1 << ROTARY_4_DIR_BIT,
                                                 1 << LINEAR_1_DIR_BIT, 1 << LINEAR_2_DIR_BIT};

    Settings            *settings;
    State               *state;
    Callbacks           *callbacks;

    // Step and direction port invert masks.
    uint16_t            step_port_invert_mask;
    uint16_t            dir_port_invert_mask;

    motion_block_t      *planned_block;

    // Step segment ring buffer indices
    segment_t           segment_buffer[SEGMENT_BUFFER_SIZE];
    volatile uint8_t    segment_buffer_tail;
    uint8_t             segment_buffer_head;
    uint8_t             segment_next_head;

    st_block_t          st_block_buffer[SEGMENT_BUFFER_SIZE-1];
    st_block_t          *st_prep_block;
    profile_prep_t      profile;
    stepper_t           st;

    float               units_remaining;
    float               segment_dt;
    volatile uint8_t    busy; // Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.

    static uint8_t  next_block_index(uint8_t block_index);
    static void     apply_amass(segment_t *segment, uint32_t cycles);
    bool            check_end_motion();
    uint16_t        axis_to_port_direction_bits(uint16_t axis_bits) const;

    bool            prepare_block();
    void            prepare_profile();
    bool            prepare_segment();

public:
    explicit        Steppers(Settings *settings, State *state, Callbacks *callbacks);

    void            init();

    float           get_realtime_rate() const;
    void            prepare_buffer();

    void            wake_up();
    void            go_idle();

    void            pulse_start();
    void            pulse_end() const;
};


#endif // STEPPER_H
