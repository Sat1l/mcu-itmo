#include "main.h"
#include <stdio.h>
#include <string.h>

#define MIN_DUTY    80u
#define MAX_DUTY    999u

static void delay_ms(uint32_t ms)
{
    uint32_t start = millis;
    while ((millis - start) < ms) { /* wait */ }
}

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

    /* 
       Baud rate 115200 @ 18MHz (PCLK1 is 18MHz due to DIV4 in system_init.c)
       18000000 / (16 * 115200) = 9.765625
       Mantissa = 9 (0x9)
       Fraction = 0.765625 * 16 = 12.25 -> 12 (0xC)
    */
    USART2->BRR = (9u << 4) | 12u; 
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
    GPIOA->CRL |= (0x44u << 0); // PA0, PA1 Enc1
    GPIOA->CRL &= ~(0xFFu << 24);
    GPIOA->CRL &= ~(0xFFu << 28);
    GPIOA->CRL |= (0x44u << 24) | (0x44u << 28); // PA6, PA7 Enc2
    GPIOC->CRH &= ~(0xFu << 20);
    GPIOC->CRH |= (0x2u << 20); // PC13 LED
}

static void tim1_pwm_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 143; 
    TIM1->ARR = 999;
    TIM1->CCMR1 |= (6u << TIM_CCMR1_OC1M_Pos) | (6u << TIM_CCMR1_OC2M_Pos);
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;
}

static void tim_encoders_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    TIM2->ARR = 0xFFFF;
    TIM2->SMCR |= (3u << TIM_SMCR_SMS_Pos);
    TIM2->CCMR1 |= (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos);
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->ARR = 0xFFFF;
    TIM3->SMCR |= (3u << TIM_SMCR_SMS_Pos);
    TIM3->CCMR1 |= (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos);
    TIM3->CR1 |= TIM_CR1_CEN;
}

extern void enable_systick(void);

int main(void)
{
    enable_systick();
    gpio_init();
    uart_init();
    tim1_pwm_init();
    tim_encoders_init();

    float kp = 0.8f;
    float ki = 0.05f;
    float kd = 1.2f;
    float integral = 0;
    float last_error = 0;
    const int16_t deadzone = 4;

    uint32_t last_telemetry = 0;
    int16_t last_motor_pos = 0;

    while (1)
    {
        int16_t target_pos = (int16_t)TIM3->CNT;
        int16_t current_pos = (int16_t)TIM2->CNT;
        float error = (float)(target_pos - current_pos);

        if (error > -deadzone && error < deadzone) error = 0;
        if (error != 0) integral += error;
        else integral *= 0.8f;
        
        if (integral > 300) integral = 300;
        if (integral < -300) integral = -300;

        float derivative = error - last_error;
        last_error = error;

        float output = (kp * error) + (ki * integral) + (kd * derivative);
        motor_set_speed((int16_t)output);

        // Telemetry loop (every 50ms)
        if ((millis - last_telemetry) > 50) {
            // Speed calculation: delta_pos per 50ms
            int16_t speed = current_pos - last_motor_pos;
            last_motor_pos = current_pos;

            char buf[128];
            // Format for SerialPlot: Target,Current,Speed,Error,Output
            sprintf(buf, "%d,%d,%d,%d,%d\r\n", target_pos, current_pos, speed, (int16_t)error, (int16_t)output);
            uart_send_string(buf);
            last_telemetry = millis;
        }

        static uint32_t last_blink = 0;
        uint32_t blink_period = (error == 0) ? 500 : 100;
        if ((millis - last_blink) > blink_period) {
            GPIOC->ODR ^= (1u << 13);
            last_blink = millis;
        }
        delay_ms(10);
    }
}
