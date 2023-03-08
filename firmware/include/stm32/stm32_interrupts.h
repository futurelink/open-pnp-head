/**
  ******************************************************************************
  * @file    stm32f1xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
 ******************************************************************************
  */

#ifndef stm32_interrupts_h
#define stm32_interrupts_h

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
void TIM2_IRQHandler();
void TIM4_IRQHandler();
void USART1_IRQHandler();
void ADC1_2_IRQHandler();
void DMA1_Channel2_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif
