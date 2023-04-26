/*
  state.h - FSM definitions
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
#define STATE_CYCLE_LIGHT           (1 << 7)
#define STATE_CYCLE_RELAY           (1 << 8)

// Pick-place cycle states
// -----------------------
#define STATE_PNP_CYCLE_NONE            0
#define STATE_PNP_CYCLE_MOVE_DOWN       (1 << 0)
#define STATE_PNP_CYCLE_WAIT_VACUUM     (1 << 1)
#define STATE_PNP_CYCLE_MOVE_UP         (1 << 2)
#define STATE_PNP_CYCLE_ROTATING        (1 << 3)
#define STATE_PNP_CYCLE_MOVE            (1 << 4)
#define STATE_PNP_CYCLE_ALARM           (1 << 5)

typedef struct {
    uint32_t        light_color;
    uint8_t         relays;          // {M100-104,M110-114}
    uint8_t         nozzle;         // Pick-Place active nozzle
    float           depth;          // Pick-Place cycle var indicates how deep the nozzle should go
    float           angle;          // Pick-Place rotate cycle var indicates rotation angle
    float           feed;           // Pick-Place or Move cycle feed rate
} fsm_params_t;

class State {
private:
    uint16_t            state;              // Tracks the current system state
    uint8_t             pick_place_state;   // Tracks pick-place cycle phases
    uint8_t             step_control;
    int32_t             *positions;

    uint8_t             active_nozzle;
    volatile uint8_t    vac_wait_time;
    float               vacuum[ROTARY_AXIS_N];

public:
    uint8_t             end_stops;
    uint8_t             relays;
    uint32_t            light_color;

    fsm_params_t        params;

    State();

    void        init();

    bool        is_state(uint16_t value) const;
    bool        has_state(uint16_t value) const;
    uint16_t    get_state() const;
    void        set_state(uint16_t st);

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

    float       get_vacuum(uint8_t nozzle) const;
    void        set_vacuum(uint8_t nozzle, float value);
};

// State access methods
inline uint16_t State::get_state() const { return this->state; }
inline bool State::has_state(uint16_t value) const { return (this->state & value) != 0; }
inline bool State::is_state(uint16_t value) const { return this->state == value; }
inline void State::set_state(uint16_t st) { this->state = st; }

inline uint8_t State::get_pick_place_state() const { return pick_place_state; }
inline void State::set_pick_place_state(uint8_t st) { pick_place_state = st; }

inline int32_t *State::get_positions() const { return (int32_t *) positions; }
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

inline float State::get_vacuum(uint8_t nozzle) const { return vacuum[nozzle]; }
inline void State::set_vacuum(uint8_t nozzle, float value) { vacuum[nozzle] = value; }

#endif // STATE_H
