/*
  settings.cpp - settings management routines
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

#include "system/settings.h"

#include <cmath>

Settings::Settings() {

}

bool Settings::load() {
    steps_per_degree = ROTARY_STEP_PER_DEGREE; // 1/16 micro-stepping

#ifdef ROCKER_HEAD
    prepare_rocker_head_curve();
#else
    steps_per_mm = LINEAR_STEP_PER_MM;
#endif

    return true;
}

#ifdef ROCKER_HEAD
void Settings::prepare_rocker_head_curve() {
    arm_length = ARM_LENGTH;
    arm_center_offset = ARM_CENTER_OFFSET;
    arm_roller_radius = ARM_ROLLER_RADIUS;

    // Pre-calculate rocker-head curve approximation segments
    // ------------------------------------------------------
    auto steps_per_quarter = STEP_PER_ROTATION / 4;
    float degrees_per_step = 90.0f / (float) steps_per_quarter;
    arm_max_travel = arm_length - arm_center_offset + arm_roller_radius;
    arm_segment_length = arm_max_travel / STEP_APPROX_SEGMENTS_N;

    uint8_t segment = 0;
    uint16_t tmp_step_count = 0;
    float tmp_sum = 0;
    auto step = 0;
    auto step_radian = (float)(M_PI / 180 * degrees_per_step);
    while (step <= steps_per_quarter) {
        step++;
        tmp_sum += (sinf(step_radian * (float) step) - ((step == 0) ? 0.0f : sinf(step_radian * (float) (step - 1)))) * arm_max_travel;

        if ((tmp_sum >= arm_segment_length) || (step == steps_per_quarter)) {
            step_increment[segment] = arm_segment_length / (float) tmp_step_count;
            tmp_step_count = 0;
            tmp_sum -= (float) arm_segment_length;
            segment++;
        }
        tmp_step_count++;
    }
}

float Settings::get_rocker_axis_position(const int32_t position_steps) const {
    float position = 0;
    uint32_t abs_position_steps = abs(position_steps);

    // Calculate first incomplete segment steps
    // ...

    // Calculate full segments
    float steps = 0;
    uint8_t segment = 0;
    for (; segment < STEP_APPROX_SEGMENTS_N; segment++) {
        auto st_inc = arm_segment_length / step_increment[segment];
        if (steps + st_inc >= (float) abs_position_steps) break;
        position += arm_segment_length;
        steps += st_inc;
    }

    // Calculate remainder
    auto remain_steps = (float) abs_position_steps - steps;
    if (remain_steps > 0) position += remain_steps * step_increment[segment];

    return position * ((position_steps < 0) ? -1.0f : 1.0f);
}

float Settings::get_rocker_axis_steps(const float position, const float destination) const {
    if (position == destination) return 0;

    auto abs_position = fabsf(position);
    auto abs_destination = fabsf(destination);
    float steps = 0;

    // The main idea of this algorithm is to place finalizing segment at the end, so whatever
    // movement is being done the incomplete segment will always be the final thing to be processed.
    uint8_t dir = abs_position < abs_destination ? 0 : 1;
    auto start = (dir ? abs_destination : abs_position);
    auto end = (dir ? abs_position : abs_destination);
    auto segment = (uint8_t) floorf(start / arm_segment_length);        // First full segment
    auto last_segment = (uint8_t) floorf(end / arm_segment_length);    // Last full segment
    auto pos = abs_position;

    // Calculate first incomplete segment steps
    if ((float) segment != start / arm_segment_length) {
        auto dist = (arm_segment_length - start + ((float) segment * arm_segment_length));
        steps += dist / step_increment[segment];
        if (dir) pos -= dist; else pos += dist;
        segment++;
    }

    // Calculate internal full-segment steps
    while (segment < last_segment) {
        steps += arm_segment_length / step_increment[segment];
        if (dir) pos -= arm_segment_length; else pos += arm_segment_length;
        segment++;
    }

    // Calculate last incomplete segment steps
    if (fabsf(abs_destination - pos) > 0) steps += fabsf(abs_destination - pos) / step_increment[last_segment];

    return steps;
}
#endif

void Settings::convert_rotary_steps_to_degrees(float *position, const int32_t *steps, uint8_t n_axis) const {
    for (uint8_t idx = 0; idx < n_axis; idx++) position[idx] = (float) steps[idx] / steps_per_degree;
}

void Settings::convert_linear_steps_to_mm(float *position, const int32_t *steps, uint8_t n_axis) const {
    for (uint8_t idx = 0; idx < n_axis; idx++)
#ifdef ROCKER_HEAD
    position[idx] = get_rocker_axis_position(steps[idx]);
#else
    position[idx] = (float) steps[idx] / (float) steps_per_mm;
#endif
}

float Settings::convert_linear_steps_to_mm(const int32_t steps) const {
#ifdef ROCKER_HEAD
    return get_rocker_axis_position(steps);
#else
    return (float) steps / (float) steps_per_mm;
#endif
}

float Settings::get_acceleration(uint8_t axis) const {
    if (axis < ROTARY_AXIS_N) return ROTARY_AXIS_ACCEL; // Rotary axis
    else return LINEAR_AXIS_ACCEL; // Linear axis
}

float Settings::get_speed(uint8_t axis) const {
    if (axis < ROTARY_AXIS_N) return ROTARY_AXIS_SPEED; // Rotary axis
    else return LINEAR_AXIS_SPEED; // Linear axis
}

float Settings::get_linear_max_travel() {
#ifdef ROCKER_HEAD
    return arm_max_travel;
#else
    return LINEAR_AXIS_MAX_TRAVEL;
#endif
}
