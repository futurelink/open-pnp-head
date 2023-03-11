/*
  vacuum.h
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
