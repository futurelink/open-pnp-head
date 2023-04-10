/*
  report.h
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

#ifndef REPORT_H
#define REPORT_H

#include "state.h"
#include "serial.h"
#include "settings.h"
#include "stepper.h"

#include "peripheral/vacuum.h"

class Report {
private:
    Serial *serial;
    State  *state;
    Settings *settings;

public:
    explicit Report(Settings *settings, Serial *serial, State *state);

    void status_message(uint8_t msg);

    void print_state(Vacuum *vacuum, uint8_t end_stops_state, uint8_t relay_state, uint32_t light_state);
    void print_line_feed();
    void print_float(float val);
    void print_uint8_base10(uint8_t n);
    void print_axis_values(const float *values, uint8_t n_axis);

};


#endif // REPORT_H
