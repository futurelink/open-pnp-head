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

    void print_state(Vacuum *vacuum, uint8_t end_stops_state);
    void print_line_feed();
    void print_uint8_base10(uint8_t n);
    void print_axis_values(const float *values, uint8_t n_axis);

};


#endif // REPORT_H
