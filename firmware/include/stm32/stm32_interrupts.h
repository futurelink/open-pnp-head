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

#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#ifdef __cplusplus
 extern "C" {
#endif

[[noreturn]] void NMI_Handler(void);
[[noreturn]] void HardFault_Handler(void);
[[noreturn]] void MemManage_Handler(void);
[[noreturn]] void BusFault_Handler(void);
[[noreturn]] void UsageFault_Handler(void);
void SVC_Handler();
void DebugMon_Handler();
void PendSV_Handler();
void SysTick_Handler();
void EXTI15_10_IRQHandler();
void TIM1_BRK_IRQHandler();
void TIM1_UP_IRQHandler();
void TIM1_TRG_COM_IRQHandler();
void TIM2_IRQHandler();
void TIM4_IRQHandler();
void USART1_IRQHandler();
void ADC1_2_IRQHandler();
void DMA1_Channel2_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif
