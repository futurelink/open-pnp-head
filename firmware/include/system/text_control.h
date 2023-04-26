/*
  text_control.h - text console control routines
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

#ifndef OPEN_PNP_HEAD_TEXT_CONTROL_H
#define OPEN_PNP_HEAD_TEXT_CONTROL_H

#include "control.h"

class TextControl : public Control {
private:
    Report          *report;
    parser_state_t  parser_state;

    uint8_t     process_relay();

    // Serial read temporary variables
    uint8_t     ch_cnt = 0;
    char        line[LINE_MAX_LENGTH];
    bool        line_too_long = false;

public:
    explicit TextControl(Settings *settings, Report *report, Motion *motion, State *state, Callbacks *callbacks);
    void init() override;

    void        read_serial_input() override;
    uint8_t     parse_serial_input();
    uint8_t     process_parameter();
    uint8_t     execute_command();
    void        report_state();
};

#endif //OPEN_PNP_HEAD_TEXT_CONTROL_H
