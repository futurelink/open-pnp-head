/*
  stm32_interrupts.h - hardware specific interrupt handlers
  Part of Grbl

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

#include "config.h"
#include "system/system.h"

#include "stm32f1xx_hal.h"
#include "stm32/stm32_interrupts.h"

extern PCD_HandleTypeDef hpcd_USB_FS;

[[noreturn]] void NMI_Handler(void) {
    while (true) {}
}

[[noreturn]] void HardFault_Handler(void) {
    while (true) {}
}

[[noreturn]] void MemManage_Handler(void) {
    while (true) {}
}

[[noreturn]] void BusFault_Handler(void) {
    while (true) {}
}

[[noreturn]] void UsageFault_Handler(void) {
    while (true) {}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {
    HAL_IncTick();
    System::heartbeat();
}

void EXTI15_10_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(LIMIT_MASK);
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    System::external_interrupt_limit();
}

void TIM3_IRQHandler() {
    System::steppers_pulse_start();
}

void TIM4_IRQHandler() {
    System::steppers_pulse_end();
}

void USART1_IRQHandler() {
    if (USART1->SR & USART_SR_RXNE) System::usart_receive(); // Receive
    if (USART1->CR1 & USART_CR1_TXEIE) System::usart_transmit(); // Transmit
}

void ADC1_2_IRQHandler() {
    if (ADC1->SR & ADC_SR_EOS) {
        System::adc_read(0, ADC1->JDR1);
        System::adc_read(1, ADC1->JDR2);
        System::adc_read(2, ADC1->JDR3);
        System::adc_read(3, ADC1->JDR4);
        ADC1->SR &= ~ADC_SR_EOC;
    }
}
