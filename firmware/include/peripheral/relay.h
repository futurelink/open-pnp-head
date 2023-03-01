#ifndef RELAY_H
#define RELAY_H

#include <cstdint>

class Relay {
public:
    void init();
    void set_state(uint8_t state);
};

#endif // RELAY_H
