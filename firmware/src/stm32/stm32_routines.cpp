/*
  stm32_routines.cpp - hardware specific routines
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

#include "system/settings.h"
#include "system/macros.h"
#include "stm32/stm32_routines.h"

#define EEPROM_START_ADDRESS    ((uint32_t) 0x0801FC00) // Last 1K page (127K offset)
#define EEPROM_PAGE_SIZE        0x400
#define TICKS_PER_MS            (F_CPU / 1000000UL)
#define RS485_TRANSMIT_DELAY    (TICKS_PER_MS * 20)

#ifdef WS8212LED
volatile uint8_t WS8212LED_Buffer[24];
volatile uint8_t WS8212LED_Sequence;
#endif

volatile uint8_t EE_Buffer[EEPROM_PAGE_SIZE];

static void SystemClock_Config();

void stm32_init() {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_Init();
    SystemClock_Config();

    // Enable all GPIOs
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config() {
    RCC_OscInitTypeDef RCC_OscInitStruct = {
            .OscillatorType = RCC_OSCILLATORTYPE_HSE,
            .HSEState = RCC_HSE_ON,
            .HSEPredivValue = RCC_HSE_PREDIV_DIV1,
            .HSIState = RCC_HSI_ON,
            .PLL = { .PLLState = RCC_PLL_ON, .PLLSource = RCC_PLLSOURCE_HSE, .PLLMUL = RCC_PLL_MUL9 }
    };
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {
            .ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2,
            .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
            .AHBCLKDivider = RCC_SYSCLK_DIV1,
            .APB1CLKDivider = RCC_HCLK_DIV4,
            .APB2CLKDivider = RCC_HCLK_DIV1
    };
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void stm32_config_timer(TIM_TypeDef* timer, uint16_t period, uint16_t prescaler, uint8_t PP) {
    TIM_HandleTypeDef h_timer = {
            .Instance = timer,
            .Init = {
                    .Prescaler = (uint32_t)(prescaler - 1),
                    .CounterMode = TIM_COUNTERMODE_UP,
                    .Period = (uint32_t)(period - 1),
                    .ClockDivision = TIM_CLOCKDIVISION_DIV1
            }
    };
    HAL_TIM_Base_Init(&h_timer);

    __HAL_TIM_CLEAR_FLAG(&h_timer, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&h_timer, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&h_timer);

    HAL_NVIC_SetPriorityGrouping((uint32_t)0x300); // NVIC_PriorityGroup_4

    auto IRQn = (IRQn_Type)0;
    if (timer == TIM1) IRQn = TIM1_UP_IRQn;
    else if (timer == TIM2) IRQn = TIM2_IRQn;
    else if (timer == TIM3) IRQn = TIM3_IRQn;
    else if (timer == TIM4) IRQn = TIM4_IRQn;
    if (IRQn != 0) {
        HAL_NVIC_SetPriority(IRQn, PP, 1);
        HAL_NVIC_EnableIRQ(IRQn);
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler() {}

/**
 * STM32 initialization functions
 */
void stm32_system_init() {

}

#ifdef WS8212LED
void stm32_light_run_pwm() {
    WS8212LED_Sequence = 0;

    TIM3->CR1 &= ~TIM_CR1_CEN;          // Disable timer
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;  // Disable DMA channel
    DMA1->IFCR = DMA_ISR_GIF2;          // Clear all flags in DMA Channel 2

    // Configure DMA Channel data length
    DMA1_Channel2->CNDTR = 24;
    DMA1_Channel2->CPAR = (uint32_t) &TIM3->CCR3;
    DMA1_Channel2->CMAR = (uint32_t) WS8212LED_Buffer;

    DMA1_Channel2->CCR |= DMA_IT_TC;    // Enable DMA transmission complete interrupt
    DMA1_Channel2->CCR |= DMA_CCR_EN;   // Enable DMA channel

    __HAL_RCC_TIM3_CLK_ENABLE();        // Enable timer clock

    TIM3->CCR3 = 0;                     // Reset CCR3, so that no data be sent
    TIM3->DIER |= TIM_DIER_UDE;         // Enable DMA update request
    TIM3->DIER |= TIM_DMA_CC3;          // Enable the TIM Output Capture/Compare 3 request
    TIM3->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_3);  // Enable the Capture compare channel
    TIM3->CR1 |= TIM_CR1_CEN;           // Enable timer

}

void stm32_light_set_color(const uint32_t color) {
    WS8212LED_Sequence = 0;
    uint8_t index = 0;

    DISABLE_IRQ
    for (int8_t bit = 23; bit >= 0; bit--) {
        if (color & (1 << bit)) WS8212LED_Buffer[index] = 70;     // PWM fill for '1'
        else WS8212LED_Buffer[index] = 15;                        // PWM fill for '0'
        index++;
    }

    stm32_light_run_pwm(); // Run PWM generation
    ENABLE_IRQ
}
#endif

void stm32_light_init() {
#ifdef WS8212LED
    // Set up PWM generation for WS2812b LEDs
    // --------------------------------------

    // Set up PWM pin as output with alternative function (TIM2 channel 1 output)
    GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = (1 << LIGHT_BIT),
            .Mode = GPIO_MODE_AF_PP,
            .Speed = GPIO_SPEED_FREQ_LOW
    };
    HAL_GPIO_Init(LIGHT_PORT, &GPIO_InitStruct);

    // Set up DMA for WS2812 LED PWM
    // -----------------------------
    __HAL_RCC_DMA1_CLK_ENABLE();
    DMA1_Channel2->CCR =
            DMA_CCR_PSIZE_0 |   // Peripheral size - Timer's CCR3 has 16 bit (half word)
            DMA_CCR_MINC |      // Memory increment
            DMA_CCR_DIR |       // Memory to peripheral
            DMA_CCR_CIRC |      // Circular buffer
            DMA_CCR_PL_1;       // High priority
    DMA1_Channel2->CNDTR = 0;

    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM3->PSC = 0;                                                          // No pre-scaler
    TIM3->ARR = (F_CPU / 800000 - 1);                                       // Duty cycle 800KHz
    TIM3->CCMR1 = 0;
    TIM3->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;    // Output mode
    TIM3->CCER = TIM_CCER_CC3E;                                             // Output enable
    TIM3->EGR = TIM_EGR_UG;                                                 // Update generation
    TIM3->DIER &= ~TIM_DIER_UDE;                                            // Disable DMA requests
    TIM3->CR1 &= ~TIM_CR1_CEN;                                              // Disable timer

    __HAL_RCC_TIM3_CLK_DISABLE();

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    stm32_light_set_color(0x000000);

#else

    GPIO_InitTypeDef GPIO_Init = {
            .Pin = (1 << LIGHT_BIT),
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Speed = GPIO_SPEED_HIGH
    };
    HAL_GPIO_Init(LIGHT_PORT, &GPIO_Init);

#endif
}

void stm32_relay_init() {
    GPIO_InitTypeDef GPIO_Init = {
            .Pin = ((1 << RELAY_0_BIT) | (1 << RELAY_1_BIT) | (1 << RELAY_2_BIT) | (1 << RELAY_3_BIT)),
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Speed = GPIO_SPEED_MEDIUM
    };
    HAL_GPIO_Init(RELAY_PORT, &GPIO_Init);
}

void stm32_stepper_init() {
    GPIO_InitTypeDef gpio = {
        .Pin = STEPPERS_DISABLE_MASK,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Speed = GPIO_SPEED_HIGH
    };
    HAL_GPIO_Init(STEPPERS_DISABLE_PORT, &gpio);

    gpio.Pin = STEP_MASK;
    HAL_GPIO_Init(STEP_PORT, &gpio);

    gpio.Pin = DIRECTION_MASK;
    HAL_GPIO_Init(DIRECTION_PORT, &gpio);

    __HAL_RCC_TIM2_CLK_ENABLE();
    stm32_config_timer(TIM2, 1, 1, 0);
    NVIC_DisableIRQ(TIM2_IRQn);

    __HAL_RCC_TIM4_CLK_ENABLE();
    stm32_config_timer(TIM4, 1, 1, 0);
    NVIC_DisableIRQ(TIM4_IRQn);
}

void stm32_limits_init() {
    GPIO_InitTypeDef gpio = {
        .Pin = LIMIT_MASK,
        .Mode = GPIO_MODE_INPUT,
        #ifdef DISABLE_LIMIT_PIN_PULL_UP
        .Pull = GPIO_NOPULL,
        #else
        .Pull = GPIO_PULLUP,
        #endif
        .Speed = GPIO_SPEED_HIGH
    };
    HAL_GPIO_Init(LIMIT_PORT, &gpio);
}

void stm32_limits_enable() {
    __HAL_GPIO_EXTI_CLEAR_IT(LIMIT_MASK);
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void stm32_limits_disable() {
    NVIC_DisableIRQ(EXTI15_10_IRQn);
}

uint16_t stm32_limits_get_state() {
    return (LIMIT_PORT->IDR & LIMIT_MASK) ^ LIMIT_MASK; // Invert inputs
}

void stm32_eeprom_flush() {
    uint32_t error;
    uint32_t nAddress = EEPROM_START_ADDRESS;
    uint16_t nSize = FLASH_PAGE_SIZE;
    auto pBuffer = (uint16_t *) EE_Buffer;

    HAL_FLASH_Unlock();

    // Erase the page allocated for settings
    FLASH_EraseInitTypeDef eraseDef = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = 2,
        .PageAddress = nAddress,
        .NbPages = 1
    };
    if(HAL_FLASHEx_Erase(&eraseDef, &error) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    // Write settings
    while (nSize > 0) {
        if (*pBuffer != 0xffff) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, nAddress, *pBuffer++);
        } else {
            pBuffer++;
        }

        if (*pBuffer != 0xffff) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, nAddress + 2, *pBuffer++);
        } else {
            pBuffer++;
        }

        nSize -= 4;
        nAddress += 4;
    }

    HAL_FLASH_Lock();
}

void stm32_eeprom_init() {
    uint16_t VarIdx;
    auto pTmp = EE_Buffer;

    HAL_FLASH_Unlock();

    // Read settings into buffer
    for (VarIdx = 0; VarIdx < FLASH_PAGE_SIZE; VarIdx++) {
        *pTmp++ = (*(__IO uint8_t*)(EEPROM_START_ADDRESS + VarIdx));
    }

    // If settings version does not match - clear buffer with 0xFF
    if (EE_Buffer[0] != SETTINGS_VERSION) {
        pTmp = EE_Buffer;
        for (VarIdx = 0; VarIdx < FLASH_PAGE_SIZE; VarIdx++) *pTmp++ = 0xFF;
    }

    HAL_FLASH_Lock();
}

uint8_t stm32_eeprom_get_char(uint32_t addr) {
    return EE_Buffer[addr];
}

void stm32_eeprom_put_char(uint32_t addr, uint8_t value) {
    EE_Buffer[addr] = value;
}

uint16_t stm32_relay_get_state() {
    return (RELAY_PORT->ODR & RELAY_MASK);
}

uint8_t stm32_relay_get_single_state(uint8_t relay) {
    auto state = stm32_relay_get_state();
    switch (relay) {
        case 0: return state & (1 << RELAY_0_BIT);
        case 1: return state & (1 << RELAY_1_BIT);
        case 2: return state & (1 << RELAY_2_BIT);
        case 3: return state & (1 << RELAY_3_BIT);
        case 4: return state & (1 << LIGHT_BIT);
        default: return 0;
    }
}

void stm32_relay_set_state(uint8_t state) {
    uint32_t value =
            ((state & 0x01) ? (1 << RELAY_0_BIT) : ((1 << RELAY_0_BIT) << 16u)) |
            ((state & 0x02) ? (1 << RELAY_1_BIT) : ((1 << RELAY_1_BIT) << 16u)) |
            ((state & 0x04) ? (1 << RELAY_2_BIT) : ((1 << RELAY_2_BIT) << 16u)) |
            ((state & 0x08) ? (1 << RELAY_3_BIT) : ((1 << RELAY_3_BIT) << 16u));
    RELAY_PORT->BSRR = value;
}

bool stm32_steppers_disabled() {
#ifdef STEPPER_ENABLE_INVERT
        return ((STEPPERS_DISABLE_PORT->ODR & STEPPERS_DISABLE_MASK) == 0);
#else
        return ((STEPPERS_DISABLE_PORT->ODR & STEPPERS_DISABLE_MASK) != 0);
#endif
}

void stm32_steppers_enable() {
    if (stm32_steppers_disabled()) {                    // Must not go enable whilst enabled(!)
                                                        // Set micro-steps on rotary axes (1/16)
        STEP_PORT->ODR |= ROTARY_STEP_MASK;             // Set MODE4
        DIRECTION_PORT->ODR |= ROTARY_DIRECTION_MASK;   // Set MODE3

#ifdef STEPPER_ENABLE_INVERT
        STEPPERS_DISABLE_PORT->BSRR = STEPPERS_DISABLE_MASK;
#else
        STEPPERS_DISABLE_PORT->BSRR = (STEPPERS_DISABLE_MASK) << 16U;
#endif

        HAL_Delay(1);                              // Wait 1ms while driver goes active
    }
}

void stm32_steppers_disable() {
#ifdef STEPPER_ENABLE_INVERT
    STEPPERS_DISABLE_PORT->BSRR = (STEPPERS_DISABLE_MASK) << 16U;
#else
    STEPPERS_DISABLE_PORT->BSRR = STEPPERS_DISABLE_MASK;
#endif
}

void stm32_steppers_pulse_end(uint16_t step_mask) {
    if ((TIM4->SR & TIM_SR_UIF) != 0) {  // check interrupt source
        TIM4->SR &= ~TIM_SR_UIF;         // clear UIF flag
        TIM4->CNT = 0;
        NVIC_DisableIRQ(TIM4_IRQn);
        STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | (step_mask & STEP_MASK);
    }
}

bool stm32_steppers_pulse_start(bool busy, uint16_t dir_bits, uint16_t step_bits) {
    if ((TIM2->SR & TIM_SR_UIF) != 0) {      // Check interrupt source
        TIM2->SR &= ~TIM_SR_UIF;             // Clear UIF flag
        TIM2->CNT = 0;
    } else return false;

    if (busy) return false; // The busy-flag is used to avoid reentering this interrupt

    // Set the direction pins a couple of nanoseconds before we step the steppers
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | (dir_bits & DIRECTION_MASK);
    TIM4->SR &= ~TIM_SR_UIF;

    // Then pulse the stepping pins
    // Output demanded state of step bits and current state of other port bits.
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | (step_bits & STEP_MASK);

    // Enable step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
    // exactly settings.pulse_microseconds microseconds, independent of the main timer pre-scale.
    NVIC_EnableIRQ(TIM4_IRQn);

    return true;
}

/**
 * Initialize step and direction port pins.
 * @param dir_bits
 * @param step_bits
 */
void stm32_steppers_set(uint16_t dir_bits, uint16_t step_bits) {
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | (step_bits & STEP_MASK);
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | (dir_bits & DIRECTION_MASK);
}

void stm32_steppers_set_timer(uint16_t value) {
    TIM2->ARR = value - 1; // Set the Auto-reload value
}

/**
 * Enable Stepper Driver Interrupt
 * @param step_pulse_time
 * @param cycles_per_tick
 */
void stm32_steppers_wake_up(uint8_t step_pulse_time, uint16_t cycles_per_tick) {
    TIM4->ARR = step_pulse_time - 1;
    TIM4->EGR = 1; // Immediate reload
    TIM4->SR &= ~TIM_SR_UIF;

    TIM2->ARR = cycles_per_tick - 1;
    TIM2->EGR = 1; // Immediate reload

    NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * Disable Stepper Driver Interrupt.
 * Allows Stepper Port Reset Interrupt to finish, if active.
 */
void stm32_steppers_go_idle() { NVIC_DisableIRQ(TIM2_IRQn); }

void stm32_rs485_init() {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef init = {
            .Pin = (1 << RS485_RW_BIT),
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_HIGH
    };
    HAL_GPIO_Init(RS485_RW_PORT, &init);

    init = { .Pin = (1 << RS485_TX_BIT), .Mode = GPIO_MODE_AF_PP, .Speed = GPIO_SPEED_HIGH };
    HAL_GPIO_Init(RS485_PORT, &init);

    init = { .Pin = (1 << RS485_RX_BIT), .Mode = GPIO_MODE_AF_INPUT, .Pull = GPIO_PULLDOWN, .Speed = GPIO_SPEED_HIGH };
    HAL_GPIO_Init(RS485_PORT, &init);

    UART_HandleTypeDef handle = {
            .Instance = USART1,
            .Init = {
                    .BaudRate = BAUD_RATE,
                    .WordLength = UART_WORDLENGTH_8B,
                    .StopBits = UART_STOPBITS_1,
                    .Parity = UART_PARITY_NONE,
                    .Mode = UART_MODE_TX_RX,
                    .OverSampling = UART_OVERSAMPLING_16
            }
    };
    HAL_UART_Init(&handle);
    USART1->CR1 |= USART_CR1_RXNEIE; // Enable RX interrupt

    RS485_RW_PORT->BSRR = (1 << RS485_RW_BIT) << 16U;

    USART1->CR1 |= (USART_CR1_UE    // UART enable
            | USART_CR1_RE          // Receiver enable
            | USART_CR1_TE);        // Transmitter enable

    USART1->CR3 |= USART_CR3_EIE;

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void stm32_rs485_silence_timer_init() {
    __HAL_RCC_TIM1_CLK_ENABLE();
    stm32_config_timer(TIM1, 1, 1, 0);
    TIM1->ARR = 2188;  // 2188 = 72000000 / 115200 * 3.5 bytes
    TIM1->PSC = 9;
    TIM1->EGR = 1;
    TIM1->SR &= ~TIM_SR_UIF;
    TIM1->CR1 = TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void stm32_rs485_silence_timer_reset() {
    TIM1->CNT = 0;
    TIM1->SR &= ~TIM_SR_UIF;
}

void stm32_rs485_start_transmission() {
    DISABLE_IRQ
    RS485_RW_PORT->BSRR = (1 << RS485_RW_BIT);                                  // Turn on TX driver
    for (uint32_t i = 0; i < RS485_TRANSMIT_DELAY; i++) asm volatile("nop");    // Delay to allow take a line
    USART1->CR1 |= USART_CR1_TXEIE;                                             // Turn on TX and interrupt
    ENABLE_IRQ
}

void stm32_rs485_stop_transmission() {
    DISABLE_IRQ
    USART1->CR1 &= ~USART_CR1_TXEIE;                                    // Turn off TX and interrupt
    RS485_RW_PORT->BSRR = (1 << RS485_RW_BIT) << 16U;                   // Turn off TX driver
    ENABLE_IRQ
}

bool stm32_rs485_transmit_byte(uint8_t byte) {
    if (USART1->SR & USART_SR_TXE) {
        USART1->DR = byte;
        while (!(USART1->SR & USART_SR_TC)) asm volatile("nop");
        return true;
    }
    return false;
}

bool stm32_rs485_receive_byte(uint8_t *byte) {
    // Clear data register on overrun or framing error
    if (USART1->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) (void) USART1->DR;
    else if (USART1->SR & USART_SR_RXNE) {
        *byte = USART1->DR & 0x00ff;
        return true;
    }
    return false;
}

void stm32_vacuum_sensors_init() {
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0); // Analog mode for PA 0
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1); // Analog mode for PA 1
    GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2); // Analog mode for PA 2
    GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3); // Analog mode for PA 3

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     // Turn on ADC clock
    RCC->CFGR &= ~RCC_CFGR_ADCPRE_0;        // ADC pre-scale set divide by 6
    RCC->CFGR |=  RCC_CFGR_ADCPRE_1;

    ADC1->CR1 |= ADC_CR1_SCAN;              // Enable scan mode
    ADC1->CR1 |= ADC_CR1_EOSIE;             // EOC after conversion
    ADC1->CR1 |= ADC_CR1_JAUTO;             // Enable injected channels auto conversion

    ADC1->CR2 = ADC_CR2_ADON;               // Turn on ADC
    //ADC1->CR2 |= ADC_CR2_CONT;            // Continuous conversion mode
    ADC1->CR2 |= ADC_CR2_EXTSEL;            // Select external trigger (SWSTART)
    ADC1->CR2 |= ADC_CR2_EXTTRIG;           // Enable external trigger

    // Sampling time: every 55.5 cycles for all 4 channels (the slowest sampling)
    ADC1->SMPR2 |= (0b111 << 0);            // Channel ADC0
    ADC1->SMPR2 |= (0b111 << 3);            // Channel ADC1
    ADC1->SMPR2 |= (0b111 << 6);            // Channel ADC2
    ADC1->SMPR2 |= (0b111 << 9);            // Channel ADC3

    ADC1->CR2 |= ADC_CR2_CAL;               // ENABLE CALIBRATION
    while(ADC1->CR2 & ADC_CR2_CAL) asm volatile("nop");

    ADC1->SQR1 = 0;                         // No regular channels
    ADC1->JSQR = (0 << ADC_JSQR_JSQ1_Pos);  // Injected channel 0
    ADC1->JSQR |= (1 << ADC_JSQR_JSQ2_Pos); // Injected channel 1
    ADC1->JSQR |= (2 << ADC_JSQR_JSQ3_Pos); // Injected channel 2
    ADC1->JSQR |= (3 << ADC_JSQR_JSQ4_Pos); // Injected channel 3
    ADC1->JSQR |= (3 << ADC_JSQR_JL_Pos);   // 4 injected channels

    HAL_NVIC_SetPriority(ADC1_2_IRQn, 3, 0);
    NVIC_EnableIRQ(ADC1_2_IRQn);
}

void stm32_vacuum_sensors_start() {
    ADC1->SR = 0;                           // Clear the status register
    ADC1->CR2 |= ADC_CR2_SWSTART;           // Start the conversion
}
