/*
  stm32_interrupts.h - hardware specific interrupt handlers
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

#include "config.h"
#include "system/system.h"

#include "stm32f1xx_hal.h"
#include "stm32/stm32_interrupts.h"

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

void TIM1_UP_IRQHandler() {
    if (TIM1->SR & TIM_SR_UIF) {
        TIM1->SR &= ~TIM_SR_UIF;
        System::silence_timer_fired();
    }

    if (TIM1->SR & TIM_SR_TIF) {
        TIM1->SR &= ~TIM_SR_TIF;
    }
}

void TIM2_IRQHandler() {
    System::steppers_pulse_start();
}

void TIM4_IRQHandler() {
    System::steppers_pulse_end();
}

void USART1_IRQHandler() {
    if (USART1->SR & USART_SR_RXNE) System::usart_receive(); // Receive
    else if (USART1->CR1 & USART_CR1_TXEIE) System::usart_transmit(); // Transmit
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

#ifdef WS8212LED
extern uint8_t WS8212LED_Sequence;
extern uint8_t WS8212LED_Buffer[];

void DMA1_Channel2_IRQHandler() {
    if (DMA1->ISR & DMA_ISR_TCIF2) {
        DMA1->IFCR = DMA_ISR_TCIF2;
        if (WS8212LED_Sequence == (WS8212LED_N - 1)) { // 16 LEDs, 15th is the last
            // Stop transmission
            // -----------------
            // 1) Disable timer immediately to stop transmission
            __HAL_RCC_TIM3_CLK_DISABLE();

            // 2) Turn off timer
            TIM3->CCER &= ~(TIM_CCx_ENABLE << TIM_CHANNEL_3);
            TIM3->DIER &= ~TIM_DMA_CC3;
            TIM3->CR1 &= ~TIM_CR1_CEN;

            // 3) Turn off DMA
            DMA1_Channel2->CCR &= ~DMA_CCR_EN;
            DMA1_Channel2->CCR &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_HT);

            WS8212LED_Sequence = 0;
        } else WS8212LED_Sequence++;
    }
}
#endif