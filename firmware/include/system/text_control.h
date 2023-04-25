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
