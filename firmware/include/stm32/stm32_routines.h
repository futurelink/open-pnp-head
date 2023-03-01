/*
  stm32_routines.h - hardware specific routines
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2022 Denis Pavlov

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "cpu_map.h"

#ifndef stm32_routines_h
#define stm32_routines_h

#ifdef __cplusplus
extern "C" {
#endif

#define F_CPU                   SystemCoreClock
#define PORTPINDEF              uint16_t

void stm32_config_timer(TIM_TypeDef *TIMER, uint16_t Period, uint16_t Prescaler, uint8_t PP);

void stm32_init();
void stm32_system_init();

void stm32_limits_init();
void stm32_limits_enable();
void stm32_limits_disable();
uint16_t stm32_limits_get_state();

void stm32_eeprom_init();
void stm32_eeprom_flush();
uint8_t stm32_eeprom_get_char(uint32_t addr);
void stm32_eeprom_put_char(uint32_t addr, uint8_t value);

void stm32_relay_init();
uint8_t stm32_get_relay_state(uint8_t relay);
void stm32_set_relay_state(uint8_t state);

void stm32_stepper_init();
void stm32_steppers_enable(bool invert);
void stm32_steppers_disable(bool invert);
void stm32_steppers_wake_up(uint8_t step_pulse_time, uint16_t cycles_per_tick, uint16_t prescaler);
void stm32_steppers_go_idle();
void stm32_steppers_set(PORTPINDEF dir_bits, PORTPINDEF step_bits);
void stm32_steppers_set_timer(uint16_t value);
bool stm32_steppers_pulse_start(bool busy, PORTPINDEF dir_bits, PORTPINDEF step_bits);
void stm32_steppers_pulse_end(PORTPINDEF step_mask);

void stm32_rs485_init();
bool stm32_rs485_transmit_byte(uint8_t byte);
bool stm32_rs485_receive_byte(uint8_t *byte);
void stm32_rs485_start_transmission();
void stm32_rs485_stop_transmission();

void stm32_vacuum_sensors_init();
void stm32_vacuum_sensors_start();


#ifdef __cplusplus
}
#endif

#endif
