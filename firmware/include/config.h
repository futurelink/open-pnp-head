/*
  config.h - main configuration
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

#include <cstdint>

#ifndef CONFIG_H
#define CONFIG_H

// Enables ModBus mode
#define MODBUS

#define BAUD_RATE           115200

#define GREETING_STRING     "PNP Head / v0.1\n"
#define LINE_MAX_LENGTH     80

// Turns on rocker head mode.
// Comment out to switch to linear movement mode
// ---------------------------------------------
#define ROCKER_HEAD

#define ROTARY_AXIS_N       4                       // Number of rotary axes i.e. nozzles
#define LINEAR_AXIS_N       (ROTARY_AXIS_N / 2)     // Number of linear/rocker axes
#define AXIS_N              (ROTARY_AXIS_N + LINEAR_AXIS_N)

// Axis direction inverted
#define AXIS_DIR_INVERT  ((1 << 0) | (1 << 1) | (2 << 1) | (3 << 1)) // All rotary axes

// Axis speed/acceleration settings
// --------------------------------
#define ROTARY_AXIS_ACCEL       25000               // deg/s^2
#define ROTARY_AXIS_SPEED       2500                // deg/s
#define ROTARY_STEP_PER_DEGREE  (200.0 * 16 / 360)  // steps/deg, 1/16 micro-step

// Vacuum sensors settings and minimum values
// ------------------------------------------
#define VAC_SENSORS_N                   4
#define VAC_SENSOR_1_MIN_PRESSURE       (-100) // kPa
#define VAC_SENSOR_2_MIN_PRESSURE       (-100) // kPa
#define VAC_SENSOR_3_MIN_PRESSURE       (-100) // kPa
#define VAC_SENSOR_4_MIN_PRESSURE       (-100) // kPa

// Number of relay outputs
// -----------------------
#define RELAY_N                         4

// Vacuum values with components sucked to nozzle
// ----------------------------------------------
#define VAC_SENSOR_1_LOCKED_PRESSURE    (-0.01) // kPa
#define VAC_SENSOR_2_LOCKED_PRESSURE    (-5) // kPa
#define VAC_SENSOR_3_LOCKED_PRESSURE    (-5) // kPa
#define VAC_SENSOR_4_LOCKED_PRESSURE    (-5) // kPa

#define STEPPER_ENABLE_INVERT
#define ACCELERATION_TICKS_PER_SECOND   50
#define STEP_PULSE_MS                   2

// Time in milliseconds to wait while pressure normalizes before measuring
// -----------------------------------------------------------------------
#define PICK_WAIT_TIME                  100  // when picking
#define PLACE_WAIT_TIME                 150  // when placing

// Rocker head settings
// --------------------
#ifdef ROCKER_HEAD
#define LINEAR_AXIS_ACCEL               4000                // mm/s^2
#define LINEAR_AXIS_SPEED               4000                // mm/s
#define STEP_PER_ROTATION               12800
#define STEP_APPROX_SEGMENTS_N          20
#define ARM_LENGTH                      31    // mm
#define ARM_CENTER_OFFSET               7       // mm
#define ARM_ROLLER_RADIUS               5       // mm
#else
#define LINEAR_AXIS_ACCEL               4000                // mm/s^2
#define LINEAR_AXIS_SPEED               4000                // mm/s
#define LINEAR_AXIS_MAX_TRAVEL          50.0F               // mm
#define LINEAR_STEP_PER_MM              160.0F              // steps/mm
#endif

// Addressable LED light settings
// ------------------------------
#define WS8212LED                               // Turns on addressable LEDs support
#define WS8212LED_N                     16      // LEDs number

#endif // CONFIG_H
