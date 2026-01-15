#include "stm32f446xx.h"
#include <stdint.h>
#include <stdio.h>

#define SYSCLK_HZ           180000000U
#define PCLK1_HZ            45000000U
#define TIM_APB1_HZ         90000000U

#define UART_BAUD           115200U
#define PWM_HZ              1000U
#define MOTOR_PPR           1000.0f 

#define DIR_PORT            GPIOB
#define DIR_PIN             5U
#define PWM_PORT            GPIOB
#define PWM_PIN             4U
#define UART_TX_PORT        GPIOA
#define UART_TX_PIN         2U

static volatile float target_rpm = 0;
static volatile float current_rpm = 0;
static volatile uint8_t pwm_pct = 0;
static volatile uint8_t dir_state = 0;
static volatile uint8_t telemetry_flag = 0;

static void clock_180mhz_hse18(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->PLLCFGR = (18U << RCC_PLLCFGR_PLLM_Pos) | (360U << RCC_PLLCFGR_PLLN_Pos) | 
                   (0U << RCC_PLLCFGR_PLLP_Pos) | RCC_PLLCFGR_PLLSRC_HSE | (7U << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}

static void usart2_send_str(const char *s) {
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *s++;
    }
}

static void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    
    // DIR & PWM
    DIR_PORT->MODER &= ~(3U << (DIR_PIN * 2U));
    DIR_PORT->MODER |= (1U << (DIR_PIN * 2U));
    PWM_PORT->MODER &= ~(3U << (PWM_PIN * 2U));
    PWM_PORT->MODER |= (2U << (PWM_PIN * 2U));
    PWM_PORT->AFR[0] &= ~(0xFU << (PWM_PIN * 4U));
    PWM_PORT->AFR[0] |= (2U << (PWM_PIN * 4U));

    // TIM2 Encoder (PA0, PA1)
    GPIOA->MODER &= ~((3U << (0 * 2U)) | (3U << (1 * 2U)));
    GPIOA->MODER |= (2U << (0 * 2U)) | (2U << (1 * 2U));
    GPIOA->AFR[0] &= ~((0xFU << (0 * 4U)) | (0xFU << (1 * 4U)));
    GPIOA->AFR[0] |= (1U << (0 * 4U)) | (1U << (1 * 4U));
    GPIOA->PUPDR |= (1U << (0 * 2U)) | (1U << (1 * 2U));

    // TIM4 Encoder (PB6, PB7)
    GPIOB->MODER &= ~((3U << (6 * 2U)) | (3U << (7 * 2U)));
    GPIOB->MODER |= (2U << (6 * 2U)) | (2U << (7 * 2U));
    GPIOB->AFR[0] &= ~((0xFU << (6 * 4U)) | (0xFU << (7 * 4U)));
    GPIOB->AFR[0] |= (2U << (6 * 4U)) | (2U << (7 * 4U));
    GPIOB->PUPDR |= (1U << (6 * 2U)) | (1U << (7 * 2U));

    // UART TX (PA2)
    UART_TX_PORT->MODER &= ~(3U << (UART_TX_PIN * 2U));
    UART_TX_PORT->MODER |= (2U << (UART_TX_PIN * 2U));
    UART_TX_PORT->AFR[0] &= ~(0xFU << (UART_TX_PIN * 4U));
    UART_TX_PORT->AFR[0] |= (7U << (UART_TX_PIN * 4U));
}

static void usart2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = (PCLK1_HZ + (UART_BAUD / 2U)) / UART_BAUD;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void pwm_set(uint8_t pct) {
    pwm_pct = (pct > 100) ? 100 : pct;
    TIM3->CCR1 = (uint32_t)((float)pwm_pct * (float)(TIM3->ARR + 1) / 100.0f);
}

static void dir_set(uint8_t dir) {
    dir_state = !!dir;
    // Инвертировал логику: теперь dir=1 -> LOW, dir=0 -> HIGH (или наоборот, если было не так)
    if (dir_state) DIR_PORT->BSRR = (1U << (DIR_PIN + 16U));
    else DIR_PORT->BSRR = (1U << DIR_PIN);
}

void TIM5_IRQHandler(void) {
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR &= ~TIM_SR_UIF;
        
        // 1. Читаем ручку
        int16_t knob = (int16_t)TIM4->CNT;
        if (knob > 300) { knob = 300; TIM4->CNT = 300; }
        else if (knob < -300) { knob = -300; TIM4->CNT = (uint32_t)(int16_t)-300; }
        target_rpm = (float)knob;

        // 2. Считаем скорость
        static int16_t last_cnt = 0;
        int16_t cur_cnt = (int16_t)TIM2->CNT;
        int16_t delta = cur_cnt - last_cnt;
        last_cnt = cur_cnt;
        float inst_rpm = ((float)delta / MOTOR_PPR) * 6000.0f;

        // 3. ПИ-регулятор
        static float integral = 0;
        static float filtered = 0;
        filtered = filtered * 0.7f + inst_rpm * 0.3f;
        
        float err = target_rpm - filtered;
        float Kp = 0.12f, Ki = 0.05f;
        
        integral += err * Ki;
        if (integral > 100) integral = 100; else if (integral < -100) integral = -100;
        
        float out = err * Kp + integral;
        if (out > 100) out = 100; else if (out < -100) out = -100;

        // 4. Мотор
        if (out >= 0) { dir_set(1); pwm_set((uint8_t)out); }
        else { dir_set(0); pwm_set((uint8_t)(-out)); }

        // 5. Телеметрия
        static uint8_t cnt = 0;
        static int16_t last_tele = 0;
        if (++cnt >= 5) {
            cnt = 0;
            current_rpm = ((float)((int16_t)TIM2->CNT - last_tele) / MOTOR_PPR) * 1200.0f;
            last_tele = (int16_t)TIM2->CNT;
            telemetry_flag = 1;
        }
    }
}

int main(void) {
    SCB->CPACR |= (3UL << (10U * 2U)) | (3UL << (11U * 2U));
    clock_180mhz_hse18();
    gpio_init();
    usart2_init();
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN;
    
    TIM3->PSC = 89; TIM3->ARR = 999;
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM3->CCER = TIM_CCER_CC1E;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    TIM2->SMCR = 3; TIM2->CCMR1 = 0x0101; TIM2->CR1 = TIM_CR1_CEN;
    TIM4->SMCR = 3; TIM4->CCMR1 = 0x0101; TIM4->CR1 = TIM_CR1_CEN;
    
    TIM5->PSC = 8999; TIM5->ARR = 99; TIM5->DIER = TIM_DIER_UIE; TIM5->CR1 = TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);

    while (1) {
        if (telemetry_flag) {
            telemetry_flag = 0;
            char b[64];
            sprintf(b, "%.1f,%.1f\r\n", current_rpm, target_rpm);
            usart2_send_str(b);
        }
    }
}
