#include "system/settings.h"

Settings::Settings() {
    steps_per_degree = 200.0 * 16 / 360; // 1/16 micro-stepping
    steps_per_mm = 160;
}

void Settings::convert_rotary_steps_to_degrees(float *position, const int32_t *steps, uint8_t n_axis) const {
    for (uint8_t idx = 0; idx < n_axis; idx++) position[idx] = (float) steps[idx] / steps_per_degree;
}

void Settings::convert_linear_steps_to_mm(float *position, const int32_t *steps, uint8_t n_axis) const {
    for (uint8_t idx = 0; idx < n_axis; idx++) position[idx] = (float) steps[idx] / (float) steps_per_mm;
}

float Settings::get_acceleration(uint8_t axis) const {
    if (axis < ROTARY_AXIS_N) return 25000; // Rotary axis
    else return 1000; // Linear axis
}

float Settings::get_speed(uint8_t axis) const {
    if (axis < ROTARY_AXIS_N) return 2500; // Rotary axis
    else return 2000; // Linear axis
}
