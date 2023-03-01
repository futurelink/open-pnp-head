#ifndef MACROS_H
#define MACROS_H

#include "config.h"
#include "cpu_map.h"

#define MAX_INT_DIGITS 6

#define max(a, b)               (((a) > (b)) ? (a) : (b))
#define min(a, b)               (((a) < (b)) ? (a) : (b))

#define TICKS_PER_MICROSECOND       (F_CPU / 1000000UL)
#define DT_SEGMENT                  (1.0f / (ACCELERATION_TICKS_PER_SECOND * 60.0f)) // min/segment
#define REQ_MM_INCREMENT_SCALAR     1.25f

#define DISABLE_IRQ             __disable_irq();
#define ENABLE_IRQ              __enable_irq();

#endif // MACROS_H
