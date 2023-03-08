#include <cstdint>

#ifndef CONFIG_H
#define CONFIG_H

#define GREETING_STRING     "PNP Head / v0.1\n"
#define LINE_MAX_LENGTH     80

#define ROCKER_HEAD

#define ROTARY_AXIS_N       4
#define LINEAR_AXIS_N       (ROTARY_AXIS_N / 2)
#define AXIS_N              (ROTARY_AXIS_N + LINEAR_AXIS_N)

// Axis speed/acceleration settings
// --------------------------------
#define ROTARY_AXIS_ACCEL       25000               // deg/s^2
#define ROTARY_AXIS_SPEED       2500                // deg/s
#define ROTARY_STEP_PER_DEGREE  (200.0 * 16 / 360)  // steps/deg, 1/16 micro-step

#define LINEAR_AXIS_ACCEL       4000                // mm/s^2
#define LINEAR_AXIS_SPEED       4000                // mm/s
#define LINEAR_AXIS_MAX_TRAVEL  50.0F               // mm
#define LINEAR_STEP_PER_MM      160.0F              // steps/mm

// Vacuum sensors settings and minimum values
// ------------------------------------------
#define VAC_SENSORS_N                   4
#define VAC_SENSOR_1_MIN_PRESSURE       (-100) // kPa
#define VAC_SENSOR_2_MIN_PRESSURE       (-100) // kPa
#define VAC_SENSOR_3_MIN_PRESSURE       (-100) // kPa
#define VAC_SENSOR_4_MIN_PRESSURE       (-100) // kPa

// Vacuum values with components sucked to nozzle
// ----------------------------------------------
#define VAC_SENSOR_1_LOCKED_PRESSURE    (-0.01) // kPa
#define VAC_SENSOR_2_LOCKED_PRESSURE    (-5) // kPa
#define VAC_SENSOR_3_LOCKED_PRESSURE    (-5) // kPa
#define VAC_SENSOR_4_LOCKED_PRESSURE    (-5) // kPa

#define STEPPER_ENABLE_INVERT           false
#define ACCELERATION_TICKS_PER_SECOND   50
#define STEP_PULSE_MS                   2

// Time in ms to wait while pressure normalizes:
#define PICK_WAIT_TIME                  50  // when picking
#define PLACE_WAIT_TIME                 70  // when placing

#ifdef ROCKER_HEAD
#define STEP_PER_ROTATION               6400
#define STEP_APPROX_SEGMENTS_N          20
#define ARM_LENGTH                      29.5    // mm
#define ARM_CENTER_OFFSET               7       // mm
#define ARM_ROLLER_RADIUS               5       // mm
#endif

#define WS8212LED                               // Turns on addressable LEDs
#define WS8212LED_N                     16      // LEDs number

#endif // CONFIG_H
