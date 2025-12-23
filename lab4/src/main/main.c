#include "stm32f446xx.h"
#include <stdint.h>
#include <math.h>

#define HSE_HZ          12000000U
#define SYSCLK_HZ       180000000U

#define UART_BAUD       1000000U
#define N_SAMPLES       256U
#define VREF_V          3.3f

#define SINE_HZ         30.0f
#define SINE_VMAX       3.0f

#define LED_PORT        GPIOA
#define LED_PIN         5U

#define DAC_PORT        GPIOA
#define DAC_PIN         4U

#define ADC_PORT        GPIOC
#define ADC_PIN         2U
#define ADC_CH          12U

static volatile uint16_t lut[N_SAMPLES];
static volatile uint32_t lut_idx = 0;

static volatile uint16_t adc_last = 0;
static volatile uint8_t  tx_flag  = 0;

static volatile uint16_t dac_last = 0;



static void FPU_Enable(void) {
    SCB->CPACR |= (3UL << (10U * 2U)) | (3UL << (11U * 2U));
}

static void SystemClock_Config_180MHz_HSE16(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    RCC->CFGR = 0;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;  // PCLK1 = 45 MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;  // PCLK2 = 90 MHz

    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= (12U  << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (360U << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (0U   << RCC_PLLCFGR_PLLP_Pos);   // PLLP=2
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
    RCC->PLLCFGR |= (7U   << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}

static void RCC_EnablePeripherals(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
}

static void GPIO_Config(void) {
    // LED PA5 output
    LED_PORT->MODER &= ~(3U << (LED_PIN * 2U));
    LED_PORT->MODER |=  (1U << (LED_PIN * 2U));

    // DAC PA4 analog
    DAC_PORT->MODER &= ~(3U << (DAC_PIN * 2U));
    DAC_PORT->MODER |=  (3U << (DAC_PIN * 2U));

    // ADC PC2 analog
    ADC_PORT->MODER &= ~(3U << (ADC_PIN * 2U));
    ADC_PORT->MODER |=  (3U << (ADC_PIN * 2U));

    // USART2 PA2/PA3 AF7
    GPIOA->MODER &= ~(0xFU << (2U * 2U));
    GPIOA->MODER |=  (0xAU << (2U * 2U));
    GPIOA->AFR[0] &= ~((0xFU << (2U * 4U)) | (0xFU << (3U * 4U)));
    GPIOA->AFR[0] |=  ((7U   << (2U * 4U)) | (7U   << (3U * 4U)));
}

static void USART2_Config(uint32_t baud) {
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    const uint32_t pclk1 = 45000000U;
    USART2->BRR = (pclk1 + (baud / 2U)) / baud;

    USART2->CR1 |= USART_CR1_TE;
    USART2->CR1 |= USART_CR1_UE;
}

static void USART2_SendByte(uint8_t b) {
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = b;
}

static void USART2_SendU16Line(uint16_t v) {
    char buf[8];
    int n = 0;

    if (v == 0) {
        buf[n++] = '0';
    } else {
        char tmp[6];
        int t = 0;
        while (v && t < (int)sizeof(tmp)) {
            tmp[t++] = (char)('0' + (v % 10U));
            v /= 10U;
        }
        while (t--) buf[n++] = tmp[t];
    }

    buf[n++] = '\n';
    for (int i = 0; i < n; i++) USART2_SendByte((uint8_t)buf[i]);
}

static void DAC1_Config(void) {
    DAC->CR &= ~DAC_CR_EN1;
    DAC->CR &= ~(DAC_CR_WAVE1 | DAC_CR_TSEL1 | DAC_CR_TEN1);
    DAC->CR &= ~DAC_CR_BOFF1;   // buffer ON
    DAC->CR |= DAC_CR_EN1;
    DAC->DHR12R1 = 0U;
}

// waveform 0..Vmax
static void build_sine_lut(float vmax_v) {
    float a = vmax_v;
    if (a < 0.0f) a = 0.0f;
    if (a > VREF_V) a = VREF_V;

    float k = (a / VREF_V) * 2047.5f; // 4095/2

    for (uint32_t i = 0; i < N_SAMPLES; i++) {
        float x = 2.0f * 3.14159265358979323846f * ((float)i / (float)N_SAMPLES);
        float y = k * (1.0f + sinf(x));      // 0..2k
        if (y < 0.0f) y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;
        lut[i] = (uint16_t)(y + 0.5f);
    }
}

static void ADC1_Config_CH12(void) {
    ADC->CCR = 0;
    ADC->CCR |= (3U << ADC_CCR_ADCPRE_Pos);  // PCLK2/8

    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    ADC1->SQR1 = 0;
    ADC1->SQR3 = (ADC_CH & 0x1FU);

    uint32_t shift = (ADC_CH - 10U) * 3U;
    ADC1->SMPR1 &= ~(7U << shift);
    ADC1->SMPR1 |=  (7U << shift);           // 480 cycles

    ADC1->CR1 |= ADC_CR1_EOCIE;
    ADC1->CR2 |= ADC_CR2_ADON;
}

static void TIM6_Config_Sampling(void) {
    const uint32_t timclk = 90000000U; // APB1 timer clock
    float fs = SINE_HZ * (float)N_SAMPLES;

    uint32_t arr = (uint32_t)((float)timclk / fs + 0.5f);
    if (arr < 2U) arr = 2U;

    TIM6->CR1 = 0;
    TIM6->PSC = 0;
    TIM6->ARR = arr - 1U;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR  = 0;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
}

static void TIM7_Config_TxRate(void) {
    const uint32_t timclk = 90000000U;
    float fs = SINE_HZ * (float)N_SAMPLES;   // send each sample

    uint32_t arr = (uint32_t)((float)timclk / fs + 0.5f);
    if (arr < 2U) arr = 2U;

    TIM7->CR1 = 0;
    TIM7->PSC = 0;
    TIM7->ARR = arr - 1U;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->SR  = 0;
    TIM7->DIER |= TIM_DIER_UIE;
    TIM7->CR1 |= TIM_CR1_CEN;
}

static void NVIC_Enable_Interrupts(void) {
    NVIC_SetPriority(TIM6_DAC_IRQn, 2);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    NVIC_SetPriority(ADC_IRQn, 1);
    NVIC_EnableIRQ(ADC_IRQn);

    NVIC_SetPriority(TIM7_IRQn, 3);
    NVIC_EnableIRQ(TIM7_IRQn);
}

void TIM6_DAC_IRQHandler(void) {
    if ((TIM6->SR & TIM_SR_UIF) == 0U) return;

    TIM6->SR &= ~TIM_SR_UIF;

    uint16_t v = lut[lut_idx];
    lut_idx++;
    if (lut_idx >= N_SAMPLES) lut_idx = 0;

    dac_last = v;
    DAC->DHR12R1 = v;

    ADC1->CR2 |= ADC_CR2_SWSTART;

    LED_PORT->ODR ^= (1U << LED_PIN);
}


void ADC_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_EOC) {
        adc_last = (uint16_t)(ADC1->DR & 0xFFFFU);
    }
}

void TIM7_IRQHandler(void) {
    if ((TIM7->SR & TIM_SR_UIF) == 0U) return;
    TIM7->SR &= ~TIM_SR_UIF;

    tx_flag = 1;
}


static void USART2_SendStr(const char *s) {
    while (*s) USART2_SendByte((uint8_t)*s++);
}

static void USART2_SendU32(uint32_t v) {
    char buf[11];
    int n = 0;

    if (v == 0) {
        USART2_SendByte('0');
        return;
    }
    while (v && n < (int)sizeof(buf)) {
        buf[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n--) USART2_SendByte((uint8_t)buf[n]);
}


int main(void) {
    FPU_Enable();
    SystemClock_Config_180MHz_HSE16();

    RCC_EnablePeripherals();
    GPIO_Config();

    build_sine_lut(SINE_VMAX);

    USART2_Config(UART_BAUD);
    USART2_SendStr("boot\n");

    DAC1_Config();
    ADC1_Config_CH12();

    NVIC_Enable_Interrupts();

    TIM6_Config_Sampling();
    TIM7_Config_TxRate();

    for (;;) {
        if (tx_flag) {
            tx_flag = 0;
            USART2_SendU16Line(adc_last);
        }
        __WFI();
    }

}
