#include "main.h"

#define MIN_DUTY    150u
#define MAX_DUTY    999u

static void delay_ms(uint32_t ms)
{
    uint32_t start = millis;
    while ((millis - start) < ms) { /* wait */ }
}

/* 
   GyverMotor2 - Advanced 2-wire logic for BTS7960
   Forward:  L=0, R=PWM (Mode 1)
   Backward: L=PWM, R=1 (Mode 2) -> creates negative voltage
*/
void motor_set_speed(int16_t speed)
{
    static int16_t last_speed = 0;
    
    // Safety: Brake before reversing direction
    if ((last_speed > 0 && speed < 0) || (last_speed < 0 && speed > 0)) {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        delay_ms(100);
    }
    last_speed = speed;

    if (speed == 0) {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        return;
    }

    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    uint32_t abs_speed = (speed > 0) ? speed : -speed;
    uint32_t duty = MIN_DUTY + (abs_speed * (MAX_DUTY - MIN_DUTY) / 100);

    if (speed > 0) {
        // Forward (Mode 1): L=0, R=PWM
        TIM1->CCR2 = 0;        // PA9
        TIM1->CCR1 = duty;     // PA8
    } else {
        // Backward (Mode 2): L=PWM, R=1
        // V_motor = V_L - V_R = duty - 1 (Negative voltage)
        TIM1->CCR1 = MAX_DUTY; // PA8
        TIM1->CCR2 = duty;     // PA9
    }
}

void motor_brake(void)
{
    // Active brake: both pins HIGH
    TIM1->CCR1 = MAX_DUTY;
    TIM1->CCR2 = MAX_DUTY;
}

static void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
    
    // PA8, PA9 -> AF Output Push-Pull
    GPIOA->CRH &= ~(0xFFu << 0);
    GPIOA->CRH |= (0xBBu << 0);

    // PC13 -> LED
    GPIOC->CRH &= ~(0xFu << 20);
    GPIOC->CRH |= (0x2u << 20);
}

static void tim1_pwm_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 71;
    TIM1->ARR = 999;

    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM1->CCMR1 |= (6u << TIM_CCMR1_OC1M_Pos) | (6u << TIM_CCMR1_OC2M_Pos);
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); 
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
}

extern void enable_systick(void);

int main(void)
{
    enable_systick();
    gpio_init();
    tim1_pwm_init();

    while (1)
    {
        // 1. Slow Forward (Speed 20)
        GPIOC->BRR = (1u << 13); // LED ON
        motor_set_speed(20);
        delay_ms(2000);

        // 2. Full Forward (Speed 100)
        motor_set_speed(100);
        delay_ms(2000);

        // 3. Active Brake
        GPIOC->BSRR = (1u << 13); // LED OFF
        motor_brake();
        delay_ms(2000);

        // 4. Backward (Speed -50)
        GPIOC->BRR = (1u << 13); // LED ON
        motor_set_speed(-50);
        delay_ms(2000);

        // 5. Stop (Passive)
        GPIOC->BSRR = (1u << 13); // LED OFF
        motor_set_speed(0);
        delay_ms(2000);
    }
}
