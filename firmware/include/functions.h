/*
  functions.h - misc utility function
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

#include <cmath>
#include <cstdint>

#ifndef FUNCTIONS_H
#define FUNCTIONS_H


class Functions {
public:

    // Simple hypotenuse computation function.
    static float hypot_f(float x, float y) { return sqrtf(x*x + y*y); }
    static bool  read_float(const char *line, uint8_t *char_counter, float *float_ptr);
};


#endif // FUNCTIONS_H
