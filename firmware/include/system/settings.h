/*
  settings.h
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

#ifndef SETTINGS_H
#define SETTINGS_H

#include "config.h"

#include <cstdint>

#define SETTINGS_VERSION 1

class Settings {
private:
    float       steps_per_degree;

    // Linear belt-driven head settings
    uint16_t    steps_per_mm;

#ifdef ROCKER_HEAD
    // Rocker-head settings
    float       arm_length;
    float       arm_center_offset;
    float       arm_roller_radius;

    // Internal variables
    float       arm_max_travel;
    float       arm_segment_length;
    float       step_increment[STEP_APPROX_SEGMENTS_N];
#endif

public:
    Settings();

    bool load();

    float get_steps_per_degree(uint8_t axis) const;
    uint16_t get_steps_per_mm(uint8_t axis) const;

    float get_acceleration(uint8_t axis) const;
    float get_speed(uint8_t axis) const;

    void convert_rotary_steps_to_degrees(float *position, const int32_t *steps, uint8_t n_axis) const;
    void convert_linear_steps_to_mm(float *position, const int32_t *steps, uint8_t n_axis) const;
    float convert_linear_steps_to_mm(int32_t steps) const;

    float get_linear_max_travel();

#ifdef ROCKER_HEAD
    void prepare_rocker_head_curve();
    float get_rocker_axis_position(int32_t position_steps) const;
    float get_rocker_axis_steps(float position, float destination) const;
#endif


};

inline float Settings::get_steps_per_degree(uint8_t axis) const { return steps_per_degree; }
inline uint16_t Settings::get_steps_per_mm(uint8_t axis) const { return steps_per_mm; }

#endif //SETTINGS_H
