#ifndef CONTROL_H
#define CONTROL_H

#include "state.h"
#include "settings.h"
#include "stepper.h"
#include "callbacks.h"
#include "report.h"
#include "parser.h"
#include "motion.h"

#include "peripheral/relay.h"
#include "peripheral/vacuum.h"
#include "peripheral/endstops.h"

#include <cstdint>

#define COMMAND_NONE        0
#define COMMAND_PICK        1
#define COMMAND_PLACE       2
#define COMMAND_ROTATE      3
#define COMMAND_MOVE        4
#define COMMAND_LIGHT       5
#define COMMAND_RELAY       6
#define COMMAND_READ_SENSOR 7

#define STATUS_OK                       0
#define STATUS_EXPECTED_COMMAND_LETTER  1
#define STATUS_BAD_NUMBER_FORMAT        2
#define STATUS_INVALID_STATEMENT        3
#define STATUS_NEGATIVE_VALUE           4
#define STATUS_BUSY                     5
#define STATUS_OVERFLOW                 11

#define STATUS_CODE_UNSUPPORTED_COMMAND        20
#define STATUS_CODE_MODAL_GROUP_VIOLATION      21
#define STATUS_CODE_UNDEFINED_FEED_RATE        22
#define STATUS_CODE_COMMAND_VALUE_NOT_INTEGER  23
#define STATUS_CODE_AXIS_COMMAND_CONFLICT      24
#define STATUS_CODE_WORD_REPEATED              25
#define STATUS_CODE_NO_AXIS_WORDS              26
#define STATUS_CODE_INVALID_LINE_NUMBER        27
#define STATUS_CODE_VALUE_WORD_MISSING         28
#define STATUS_CODE_UNSUPPORTED_COORD_SYS      29
#define STATUS_CODE_G53_INVALID_MOTION_MODE    30
#define STATUS_CODE_AXIS_WORDS_EXIST           31
#define STATUS_CODE_NO_AXIS_WORDS_IN_PLANE     32
#define STATUS_CODE_INVALID_TARGET             33
#define STATUS_CODE_ARC_RADIUS_ERROR           34
#define STATUS_CODE_NO_OFFSETS_IN_PLANE        35
#define STATUS_CODE_UNUSED_WORDS               36
#define STATUS_CODE_G43_DYNAMIC_AXIS_ERROR     37
#define STATUS_CODE_MAX_VALUE_EXCEEDED         38

class Control {
private:
    State           *state;
    Settings        *settings;
    Report          *report;
    Motion          *motion;

    Steppers        *steppers;
    Relay           *relay;
    EndStops        *end_stops;
    Vacuum          *vacuum;

    parser_state_t  parser_state;

    uint8_t     process_relay();
    void        check_and_disable_steppers();

public:
    explicit    Control(Settings *settings, Report *report, Motion *motion, State *state, Callbacks *callbacks);
    void        init();

    State       *get_state();

    uint8_t     parse_line(char *line);
    uint8_t     process_parameter();

    uint8_t     homing();
    uint8_t     execute_command();
    void        execute_realtime();

    void        report_state();
    void        end_stops_interrupt();
    void        steppers_start();
    void        steppers_pulse_start();
    void        steppers_pulse_end();
    void        vacuum_set_value(uint8_t channel, uint16_t value);
    void        vacuum_refresh();
    void        vacuum_timer_decrement();
};

inline State *Control::get_state() { return state; }
inline void Control::vacuum_timer_decrement() { state->vac_wait_timer_decrement(); }

#endif // CONTROL_H
