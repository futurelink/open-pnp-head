/*
  vacuum.cpp - vacuum sensing specific functions
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