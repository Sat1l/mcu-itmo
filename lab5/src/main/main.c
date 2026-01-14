#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#define CPU_HZ              180000000UL
#define APB1_HZ             (CPU_HZ / 4UL)
#define TIM_APB1_TIMCLK_HZ  (APB1_HZ * 2UL)

#define UART_BAUD           1000000UL

#define PWM_HZ              1000UL
#define PWM_ARR             (TIM_APB1_TIMCLK_HZ / PWM_HZ - 1UL)

#define CTRL_HZ             1000UL
#define TIM6_PSC            89UL
#define TIM6_ARR            999UL

static uint16_t adc_val_ext = 0;
static uint16_t adc_val_motor = 0;

static char tx_buf[128];

static volatile float target_deg = 0.0f;
static volatile float motor_deg  = 0.0f;
static volatile float err_deg    = 0.0f;
static volatile int16_t control  = 0;

static float ext_f = 0.0f;
static float mot_f = 0.0f;

static void clock_180mhz_hsi_pll(void) {
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    RCC->CFGR = 0;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {}

    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= (8UL   << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (360UL << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (1UL   << RCC_PLLCFGR_PLLP_Pos); // 01 = /4
    RCC->PLLCFGR |= (8UL   << RCC_PLLCFGR_PLLQ_Pos);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) {}
}

static void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    GPIOA->MODER &= ~((3UL << (2U*2U)) | (3UL << (3U*2U)));
    GPIOA->MODER |=  ((2UL << (2U*2U)) | (2UL << (3U*2U)));
    GPIOA->AFR[0] &= ~((0xFUL << (2U*4U)) | (0xFUL << (3U*4U)));
    GPIOA->AFR[0] |=  ((7UL   << (2U*4U)) | (7UL   << (3U*4U)));

    GPIOB->MODER &= ~(3UL << (10U*2U));
    GPIOB->MODER |=  (2UL << (10U*2U));
    GPIOB->AFR[1] &= ~(0xFUL << ((10U-8U)*4U));
    GPIOB->AFR[1] |=  (1UL   << ((10U-8U)*4U));

    GPIOA->MODER &= ~(3UL << (8U*2U));
    GPIOA->MODER |=  (1UL << (8U*2U));
    GPIOA->BSRR = (1UL << (8U+16U));

    GPIOB->MODER |= (3UL << (0U*2U));
    GPIOC->MODER |= (3UL << (0U*2U));
}

static void tim2_pwm_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 0;
    TIM2->ARR = PWM_ARR;

    TIM2->CCMR2 = (TIM2->CCMR2 & ~(7UL << 4)) | (6UL << 4) | TIM_CCMR2_OC3PE;
    TIM2->CCER |= TIM_CCER_CC3E;

    TIM2->CCR3 = 0;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void adc1_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC->CCR &= ~ADC_CCR_ADCPRE_Msk;
    ADC->CCR |=  ADC_CCR_ADCPRE_0; // ADCCLK = 90MHz / 4 = 22.5MHz (max 36MHz)

    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;

    // Sample time 56 cycles for IN8 (PB0) and IN10 (PC0)
    ADC1->SMPR2 |= (3UL << (8 * 3));  // Ch 8
    ADC1->SMPR1 |= (3UL << (0 * 3));  // Ch 10
}

static uint16_t adc_read(uint32_t channel) {
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC)) {}
    return (uint16_t)ADC1->DR;
}

static void usart2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    USART2->BRR = (uint16_t)((APB1_HZ + (UART_BAUD / 2UL)) / UART_BAUD);
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart2_send(const char *s, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = (uint8_t)s[i];
    }
}

static void tim6_init_1khz(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    TIM6->PSC = TIM6_PSC;
    TIM6->ARR = TIM6_ARR;

    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, 5);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    TIM6->CR1 |= TIM_CR1_CEN;
}

static void motor_set(int16_t spd) {
    if (spd > 100) spd = 100;
    if (spd < -100) spd = -100;

    if (spd >= 0) GPIOA->BSRR = (1UL << 8);
    else          GPIOA->BSRR = (1UL << (8 + 16));

    uint32_t mag = (uint32_t)(spd >= 0 ? spd : -spd);

    if (mag == 0) {
        TIM2->CCR3 = 0;
        return;
    }

    if (mag < 12) mag = 12; // min duty to move motor

    uint32_t duty = (mag * (PWM_ARR + 1UL)) / 100UL;
    if (duty > PWM_ARR) duty = PWM_ARR;

    TIM2->CCR3 = duty;
}

static uint16_t u32_to_dec(char *p, uint32_t v) {
    char t[10];
    uint16_t n = 0;
    if (v == 0) { p[0] = '0'; return 1; }
    while (v) { t[n++] = (char)('0' + (v % 10U)); v /= 10U; }
    for (uint16_t i = 0; i < n; i++) p[i] = t[n - 1U - i];
    return n;
}

static void telemetry_send(void) {
    char *p = tx_buf;
    uint16_t n = 0;

    n += u32_to_dec(p + n, adc_val_ext); p[n++] = ',';
    n += u32_to_dec(p + n, adc_val_motor);

    p[n++] = '\r';
    p[n++] = '\n';

    uart2_send(tx_buf, n);
}

static void ctrl_step(void) {
    adc_val_ext = adc_read(8);
    adc_val_motor = adc_read(10);

    float ext = ((float)adc_val_ext   * 270.0f) / 4095.0f;
    float mot = ((float)adc_val_motor * 270.0f) / 4095.0f;

    // Invert target (standard for our setup)
    ext = 270.0f - ext;

    ext_f += 0.05f * (ext - ext_f);
    mot_f += 0.05f * (mot - mot_f);

    target_deg = ext_f;
    motor_deg  = mot_f;

    err_deg = target_deg - motor_deg;

    float out = 0.0f;

    float ae = fabsf(err_deg);
    if (ae < 1.0f) { // deadzone
        out = 0.0f;
    } else {
        if (ae > 50.0f) out = 100.0f;
        else out = ae * 2.0f;
        out = copysignf(out, err_deg);
    }

    if (motor_deg <= 5.0f && out < 0.0f) out = 0.0f;
    if (motor_deg >= 265.0f && out > 0.0f) out = 0.0f;

    control = (int16_t)lroundf(out);
    motor_set(control);
}

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR &= ~TIM_SR_UIF;

        ctrl_step();

        static uint32_t div = 0;
        div++;
        if (div >= 10) {
            div = 0;
            telemetry_send();
        }
    }
}

int main(void) {
    SCB->CPACR |= (3UL << (10U*2U)) | (3UL << (11U*2U));

    clock_180mhz_hsi_pll();
    gpio_init();
    usart2_init();
    tim2_pwm_init();
    adc1_init();
    tim6_init_1khz();

    __enable_irq();

    while (1) {
        __WFI();
    }
}
