#ifndef ENDSTOPS_H
#define ENDSTOPS_H

#include <cstdint>

class EndStops {
public:
    void init();

    void interrupt();

    uint8_t get_state();
    bool all_at_zero();
};

#endif // ENDSTOPS_H
