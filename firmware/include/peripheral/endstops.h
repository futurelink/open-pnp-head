#ifndef ENDSTOPS_H
#define ENDSTOPS_H

#include <cstdint>

class EndStops {
public:
    void init();

    void interrupt();

    uint8_t get_state();
};

#endif // ENDSTOPS_H
