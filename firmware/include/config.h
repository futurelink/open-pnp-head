#include <cstdint>

#ifndef CONFIG_H
#define CONFIG_H

#define GREETING_STRING     "PNP Head / v0.1\n"
#define LINE_MAX_LENGTH     80

#define ROTARY_AXIS_N       4
#define LINEAR_AXIS_N       2
#define AXIS_N              (ROTARY_AXIS_N + LINEAR_AXIS_N)

#define VAC_SENSORS_N                   4

// Vacuum sensors minimum values
// -----------------------------
#define VAC_SENSOR_1_MIN_PRESSURE       (-100)
#define VAC_SENSOR_2_MIN_PRESSURE       (-100)
#define VAC_SENSOR_3_MIN_PRESSURE       (-100)
#define VAC_SENSOR_4_MIN_PRESSURE       (-100)

// Vacuum values with components sucked to nozzle
// ----------------------------------------------
#define VAC_SENSOR_1_LOCKED_PRESSURE    (-0.01)
#define VAC_SENSOR_2_LOCKED_PRESSURE    (-5)
#define VAC_SENSOR_3_LOCKED_PRESSURE    (-5)
#define VAC_SENSOR_4_LOCKED_PRESSURE    (-5)

#define STEPPER_ENABLE_INVERT           true
#define ACCELERATION_TICKS_PER_SECOND   50
#define STEP_PULSE_MS                   2

// Time in ms to wait while pressure normalizes:
#define PICK_WAIT_TIME                  50  // when picking
#define PLACE_WAIT_TIME                 50  // when placing

#endif // CONFIG_H
