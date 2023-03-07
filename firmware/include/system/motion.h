#ifndef MOTION_H
#define MOTION_H

#include "config.h"
#include "settings.h"
#include "state.h"

#include <cstdlib>
#include <cstdint>

typedef struct {
    float acceleration;   // Axis-limit adjusted line acceleration in (mm/min^2). Does not change.
    float units;          // The remaining distance for this block to be executed in (mm). Always positive.
    float speed;

    uint32_t steps[AXIS_N];     // Step count along each axis
    uint32_t step_event_count;  // The maximum step axis count and number of steps required to complete this block
    uint16_t direction_bits;    // The direction bit set for this block (0 - 1st axis, 1 - 2nd axis etc)
} motion_block_t;

class Motion {
private:
    Settings    *settings;
    State       *state;

    motion_block_t *current_block;

public:
    explicit Motion(Settings *settings, State *state);

    bool check_nozzle_in_position(uint8_t nozzle);
    void move(uint8_t nozzle, float depth, float speed);
    void rotate(uint8_t nozzle, float angle, float speed);

    motion_block_t *get_current_block();
    void discard_current_block();
};


#endif // MOTION_H
