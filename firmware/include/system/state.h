#ifndef STATE_H
#define STATE_H

#include "config.h"
#include "macros.h"

#include <cstdint>

// General system states
// ---------------------
#define STATE_IDLE                  0        // Must be zero. No flags.
#define STATE_ALARM                 (1 << 0) // In alarm state. Locks out all processes. Allows settings access.
#define STATE_HOMING                (1 << 1) // Performing homing cycle
#define STATE_CYCLE_PICK            (1 << 2) // Cycle is running to pick up a component
#define STATE_CYCLE_PLACE           (1 << 3) // Cycle is running to place a component
#define STATE_CYCLE_ROTATE          (1 << 4) // Cycle is running to rotate a component
#define STATE_CYCLE_MOVE            (1 << 5) // Cycle is running to move a nozzle to specific position
#define STATE_CYCLE_STOP            (1 << 6)

// Pick-place cycle states
// -----------------------
#define STATE_PNP_CYCLE_NONE            0
#define STATE_PNP_CYCLE_MOVE_DOWN       (1 << 0)
#define STATE_PNP_CYCLE_WAIT_VACUUM     (1 << 1)
#define STATE_PNP_CYCLE_MOVE_UP         (1 << 2)
#define STATE_PNP_CYCLE_ROTATING        (1 << 3)
#define STATE_PNP_CYCLE_MOVE            (1 << 4)
#define STATE_PNP_CYCLE_ALARM           (1 << 5)

class State {
private:
    uint8_t             state;              // Tracks the current system state
    uint8_t             pick_place_state;   // Tracks pick-place cycle phases
    uint8_t             step_control;
    int32_t             *positions;

    uint8_t             active_nozzle;
    volatile uint8_t    vac_wait_time;

public:
    State();

    void        init();

    bool        is_state(uint8_t value) const;
    bool        has_state(uint8_t value) const;
    uint8_t     get_state() const;
    void        set_state(uint8_t st);

    void        set_pick_place_state(uint8_t st);
    uint8_t     get_pick_place_state() const;

    int32_t     *get_positions() const;

    void        increment_position(uint8_t axis);
    void        decrement_position(uint8_t axis);

    void        set_end_motion();
    void        normal_motion();
    bool        is_end_motion() const;

    void        vac_wait_timer_decrement();
    void        set_vac_wait_timer(uint8_t value);
    bool        is_vac_wait_timeout() const;

    void        set_active_nozzle(uint8_t nozzle);
    uint8_t     get_active_nozzle() const;
};

// State access methods
inline uint8_t State::get_state() const { return this->state; }
inline bool State::has_state(uint8_t value) const { return (this->state & value) != 0; }
inline bool State::is_state(uint8_t value) const { return this->state == value; }
inline void State::set_state(uint8_t st) { this->state = st; }

inline uint8_t State::get_pick_place_state() const { return pick_place_state; }
inline void State::set_pick_place_state(uint8_t st) { pick_place_state = st; }

inline int32_t *State::get_positions() const { return positions; }
inline void State::increment_position(uint8_t axis) { positions[axis]++; }
inline void State::decrement_position(uint8_t axis) { positions[axis]--; }

inline void State::normal_motion() { step_control = 0; }
inline bool State::is_end_motion() const { return (step_control & 1); }
inline void State::set_end_motion() { step_control |= 1; }

inline void State::vac_wait_timer_decrement() { if (vac_wait_time > 0) vac_wait_time--; }
inline void State::set_vac_wait_timer(uint8_t value) { vac_wait_time = value; }
inline bool State::is_vac_wait_timeout() const { return vac_wait_time == 0; }

inline void State::set_active_nozzle(uint8_t nozzle) { active_nozzle = nozzle; }
inline uint8_t State::get_active_nozzle() const { return active_nozzle; }

#endif // STATE_H
