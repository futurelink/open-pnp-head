#ifndef SETTINGS_H
#define SETTINGS_H

#include "config.h"

#include <cstdint>

#define SETTINGS_VERSION 1

class Settings {
private:
    float       steps_per_degree;
    uint16_t    steps_per_mm;

public:
    Settings();

    float get_steps_per_degree(uint8_t axis) const;
    uint16_t get_steps_per_mm(uint8_t axis) const;

    float get_acceleration(uint8_t axis) const;
    float get_speed(uint8_t axis) const;

    void convert_rotary_steps_to_degrees(float *position, const int32_t *steps, uint8_t n_axis) const;
    void convert_linear_steps_to_mm(float *position, const int32_t *steps, uint8_t n_axis) const;

};

inline float Settings::get_steps_per_degree(uint8_t axis) const { return steps_per_degree; }
inline uint16_t Settings::get_steps_per_mm(uint8_t axis) const { return steps_per_mm; }

#endif //SETTINGS_H
