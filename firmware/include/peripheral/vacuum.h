#ifndef VACUUM_H
#define VACUUM_H

#include "config.h"

#include <cstdint>

class Vacuum {
private:
    uint16_t    values[VAC_SENSORS_N];
    float       sensor_min_pressure[VAC_SENSORS_N];
    float       sensor_locked_value[VAC_SENSORS_N];

public:
    void init();
    void refresh();
    void set_value(uint8_t channel, uint16_t value);
    float get_value(uint8_t channel);

    bool has_component(uint8_t nozzle);
};

#endif // VACUUM_H
