/*
  callbacks.h - callback definition structure
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

#ifndef CALLBACKS_H
#define CALLBACKS_H

#include "motion.h"
#include "parser.h"

typedef struct {
    motion_block_t  *(*get_current_block)();
    void            (*discard_current_block)();
} Callbacks;

#endif // CALLBACKS_H
