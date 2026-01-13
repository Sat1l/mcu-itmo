#include "main.h"
#include <stdio.h>
#include <string.h>

#define MIN_DUTY    80u
#define MAX_DUTY    999u

// Глобальные переменные для взаимодействия с прерыванием
volatile int16_t g_target_pos = 0;
volatile int16_t g_current_pos = 0;
volatile int16_t g_speed = 0;
volatile int16_t g_error = 0;
volatile int16_t g_output = 0;
volatile uint8_t g_send_telemetry = 0;

void motor_set_speed(int16_t speed)
{
    if (speed == 0) {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        return;
    }
    if (speed > 60) speed = 60;
    if (speed < -60) speed = -60;

    uint32_t abs_speed = (speed > 0) ? speed : -speed;
    uint32_t duty = MIN_DUTY + (abs_speed * (MAX_DUTY - MIN_DUTY) / 100);

    if (speed > 0) {
        TIM1->CCR1 = duty;
        TIM1->CCR2 = 0;
    } else {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = duty;
    }
}

static void uart_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRL &= ~(0xFFu << 8);
    GPIOA->CRL |= (0xBu << 8) | (0x4u << 12);
    USART2->BRR = (9u << 4) | 12u; // 115200 @ 18MHz
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart_send_string(const char *s)
{
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *s++;
    }
}

static void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
    GPIOA->CRH &= ~(0xFFu << 0);
    GPIOA->CRH |= (0xBBu << 0); // PA8, PA9 PWM
    GPIOA->CRL &= ~(0xFFu << 0);
    GPIOA->CRL |= (0x44u << 0); // PA0, PA1 Enc1 (TIM2)
    GPIOA->CRL &= ~(0xFFu << 24);
    GPIOA->CRL &= ~(0xFFu << 28);
    GPIOA->CRL |= (0x44u << 24) | (0x44u << 28); // PA6, PA7 Enc2 (TIM3)
    GPIOC->CRH &= ~(0xFu << 20);
    GPIOC->CRH |= (0x2u << 20); // PC13 LED
}

static void tim_init_all(void)
{
    // 1. TIM1 - PWM @ 1kHz
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 71; 
    TIM1->ARR = 999;
    TIM1->CCMR1 |= (6u << TIM_CCMR1_OC1M_Pos) | (6u << TIM_CCMR1_OC2M_Pos);
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;

    // 2. TIM2 & TIM3 - Encoders
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    TIM2->ARR = 0xFFFF;
    TIM2->SMCR |= (3u << TIM_SMCR_SMS_Pos);
    TIM2->CCMR1 |= (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos);
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->ARR = 0xFFFF;
    TIM3->SMCR |= (3u << TIM_SMCR_SMS_Pos);
    TIM3->CCMR1 |= (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos);
    TIM3->CR1 |= TIM_CR1_CEN;

    // 3. TIM4 - Control & Telemetry Timer @ 100Hz (10ms)
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 7199; // 72MHz / 7200 = 10,000 Hz ticks
    TIM4->ARR = 99;   // 10,000 / 100 = 100 Hz (10ms)
    TIM4->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM4->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM4_IRQn);
}

// Прерывание таймера TIM4 (выполняется каждые 10 мс)
void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF; // Сброс флага

        static int16_t last_pos = 0;
        static uint8_t telemetry_cnt = 0;
        
        // PID Parameters
        static float kp = 0.8f, ki = 0.05f, kd = 1.2f;
        static float integral = 0, last_error = 0;

        // 1. Читаем данные
        g_target_pos = (int16_t)TIM3->CNT;
        g_current_pos = (int16_t)TIM2->CNT;
        
        // 2. Считаем скорость
        g_speed = g_current_pos - last_pos;
        last_pos = g_current_pos;

        // 3. ПИД-регулятор
        float err = (float)(g_target_pos - g_current_pos);
        if (err > -4 && err < 4) err = 0; // Deadzone
        
        if (err != 0) integral += err;
        else integral *= 0.8f;
        if (integral > 300) integral = 300;
        if (integral < -300) integral = -300;

        float derivative = err - last_error;
        last_error = err;

        g_error = (int16_t)err;
        g_output = (int16_t)((kp * err) + (ki * integral) + (kd * derivative));
        
        // 4. Управление мотором
        motor_set_speed(g_output);

        // 5. Флаг телеметрии каждые 50мс (каждый 5-й вызов)
        if (++telemetry_cnt >= 5) {
            telemetry_cnt = 0;
            g_send_telemetry = 1;
        }
    }
}

extern void enable_systick(void);

int main(void)
{
    enable_systick();
    gpio_init();
    uart_init();
    tim_init_all();

    while (1)
    {
        if (g_send_telemetry) {
            g_send_telemetry = 0;
            char buf[128];
            sprintf(buf, "%d,%d,%d,%d,%d\r\n", g_target_pos, g_current_pos, g_speed, g_error, g_output);
            uart_send_string(buf);
            
            // LED Heartbeat inside telemetry sync
            static uint32_t last_blink = 0;
            uint32_t blink_p = (g_error == 0) ? 500 : 100;
            if ((millis - last_blink) > blink_p) {
                GPIOC->ODR ^= (1u << 13);
                last_blink = millis;
            }
        }
    }
}
