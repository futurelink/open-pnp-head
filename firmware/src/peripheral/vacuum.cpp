#include "config.h"
#include "peripheral/vacuum.h"
#include "stm32/stm32_routines.h"

void Vacuum::init() {
    stm32_vacuum_sensors_init();
    sensor_min_pressure[0] = VAC_SENSOR_1_MIN_PRESSURE;
    sensor_min_pressure[1] = VAC_SENSOR_2_MIN_PRESSURE;
    sensor_min_pressure[2] = VAC_SENSOR_3_MIN_PRESSURE;
    sensor_min_pressure[3] = VAC_SENSOR_4_MIN_PRESSURE;

    sensor_locked_value[0] = VAC_SENSOR_1_LOCKED_PRESSURE;
    sensor_locked_value[1] = VAC_SENSOR_2_LOCKED_PRESSURE;
    sensor_locked_value[2] = VAC_SENSOR_3_LOCKED_PRESSURE;
    sensor_locked_value[3] = VAC_SENSOR_4_LOCKED_PRESSURE;
}

void Vacuum::set_value(uint8_t nozzle, uint16_t val) {
    this->values[nozzle] = val;
}

float Vacuum::get_value(uint8_t nozzle) {
    return (float)sensor_min_pressure[nozzle] +
        ((float)values[nozzle] * -(float)sensor_min_pressure[nozzle] / 4096); // 12-bit ADC
}

void Vacuum::refresh() {
    stm32_vacuum_sensors_start();
}

bool Vacuum::has_component(uint8_t nozzle) {
    return this->get_value(nozzle) <= this->sensor_locked_value[nozzle];
}