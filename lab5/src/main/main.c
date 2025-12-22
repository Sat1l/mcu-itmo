#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#define SYSCLK_HZ           180000000U
#define APB1_PRESC          4U
#define PCLK1_HZ            (SYSCLK_HZ / APB1_PRESC)
#define TIM_APB1_HZ         (PCLK1_HZ * 2U)

#define UART_BAUD           1000000U

#define PWM_HZ              1000U
#define TELEMETRY_HZ        1000U

#define RX_DMA_BUF_SZ       256U
#define TX_DMA_BUF_SZ       256U

#define ADC_DMA_CHS         2U

static volatile uint16_t adc_dma[ADC_DMA_CHS];
static volatile uint8_t  rx_dma[RX_DMA_BUF_SZ];

static volatile uint32_t rx_last_pos = 0;

static volatile uint8_t  pwm_percent = 0;
static volatile uint8_t  dir_state   = 0;

static volatile uint8_t  tx_busy = 0;
static uint8_t tx_buf[TX_DMA_BUF_SZ];

static void clock_180mhz_hse16(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    RCC->CFGR = 0;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= (16U  << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (360U << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (0U   << RCC_PLLCFGR_PLLP_Pos);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
    RCC->PLLCFGR |= (7U   << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}

static void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

    // PA0 TIM2_CH1 (AF1) PWM
    GPIOA->MODER &= ~(3U << (0U * 2U));
    GPIOA->MODER |=  (2U << (0U * 2U));
    GPIOA->AFR[0] &= ~(0xFU << (0U * 4U));
    GPIOA->AFR[0] |=  (1U   << (0U * 4U));

    // PA1 DIR1 (output)
    GPIOA->MODER &= ~(3U << (1U * 2U));
    GPIOA->MODER |=  (1U << (1U * 2U));
    GPIOA->OTYPER &= ~(1U << 1U);

    // PA2/PA3 USART2 AF7
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER &= ~(3U << (3U * 2U));
    GPIOA->MODER |=  (2U << (2U * 2U)) | (2U << (3U * 2U));
    GPIOA->AFR[0] &= ~((0xFU << (2U * 4U)) | (0xFU << (3U * 4U)));
    GPIOA->AFR[0] |=  ((7U   << (2U * 4U)) | (7U   << (3U * 4U)));

    // PC2, PC3 analog (ADC)
    GPIOC->MODER |= (3U << (2U * 2U)) | (3U << (3U * 2U));
    GPIOC->PUPDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
}

static void tim2_pwm_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    uint32_t psc = (TIM_APB1_HZ / 1000000U) - 1U; // 1 MHz timer tick
    uint32_t arr = (1000000U / PWM_HZ) - 1U;      // 1 kHz

    TIM2->CR1 = 0;
    TIM2->PSC = psc;
    TIM2->ARR = arr;

    TIM2->CCMR1 = 0;
    TIM2->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

    TIM2->CCER = 0;
    TIM2->CCER |= TIM_CCER_CC1E;

    TIM2->CCR1 = 0;

    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void pwm_set_percent(uint8_t pct) {
    if (pct > 100U) pct = 100U;
    pwm_percent = pct;
    uint32_t arr = TIM2->ARR;
    uint32_t ccr = (pct * (arr + 1U)) / 100U;
    if (ccr > arr) ccr = arr;
    TIM2->CCR1 = ccr;
}

static void dir_set(uint8_t dir) {
    dir_state = (dir ? 1U : 0U);
    if (dir_state) GPIOA->BSRR = (1U << 1U);
    else           GPIOA->BSRR = (1U << (1U + 16U));
}

static void adc1_dma_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    ADC->CCR = 0;
    ADC->CCR |= (3U << ADC_CCR_ADCPRE_Pos);

    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    ADC1->SMPR1 = 0;
    ADC1->SMPR1 |= (4U << ADC_SMPR1_SMP12_Pos);
    ADC1->SMPR1 |= (4U << ADC_SMPR1_SMP13_Pos);

    ADC1->SQR1 = 0;
    ADC1->SQR1 |= (1U << ADC_SQR1_L_Pos);

    ADC1->SQR3 = 0;
    ADC1->SQR3 |= (12U << 0);
    ADC1->SQR3 |= (13U << 5);

    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}

    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_dma;
    DMA2_Stream0->NDTR = ADC_DMA_CHS;

    DMA2_Stream0->CR = 0;
    DMA2_Stream0->CR |= (0U << DMA_SxCR_CHSEL_Pos);
    DMA2_Stream0->CR |= DMA_SxCR_PL_1;
    DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0;
    DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0;
    DMA2_Stream0->CR |= DMA_SxCR_MINC;
    DMA2_Stream0->CR |= DMA_SxCR_CIRC;
    DMA2_Stream0->CR |= (0U << DMA_SxCR_DIR_Pos);

    DMA2_Stream0->CR |= DMA_SxCR_EN;

    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;
    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_ADON;

    for (volatile uint32_t i = 0; i < 10000U; i++) {}

    ADC1->CR2 |= ADC_CR2_SWSTART;
}

static void usart2_dma_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    uint32_t brr = (PCLK1_HZ + (UART_BAUD / 2U)) / UART_BAUD;
    USART2->BRR = brr;

    // RX DMA: DMA1 Stream5 Channel4 (USART2_RX)
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) {}

    DMA1_Stream5->PAR  = (uint32_t)&USART2->DR;
    DMA1_Stream5->M0AR = (uint32_t)rx_dma;
    DMA1_Stream5->NDTR = RX_DMA_BUF_SZ;

    DMA1_Stream5->CR = 0;
    DMA1_Stream5->CR |= (4U << DMA_SxCR_CHSEL_Pos);
    DMA1_Stream5->CR |= DMA_SxCR_MINC;
    DMA1_Stream5->CR |= DMA_SxCR_CIRC;
    DMA1_Stream5->CR |= (0U << DMA_SxCR_DIR_Pos);
    DMA1_Stream5->CR |= DMA_SxCR_PL_1;

    DMA1_Stream5->CR |= DMA_SxCR_EN;

    // TX DMA: DMA1 Stream6 Channel4 (USART2_TX)
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream6->CR & DMA_SxCR_EN) {}

    DMA1_Stream6->PAR  = (uint32_t)&USART2->DR;
    DMA1_Stream6->M0AR = (uint32_t)tx_buf;

    DMA1_Stream6->CR = 0;
    DMA1_Stream6->CR |= (4U << DMA_SxCR_CHSEL_Pos);
    DMA1_Stream6->CR |= DMA_SxCR_MINC;
    DMA1_Stream6->CR |= (1U << DMA_SxCR_DIR_Pos);
    DMA1_Stream6->CR |= DMA_SxCR_PL_1;
    DMA1_Stream6->CR |= DMA_SxCR_TCIE;

    USART2->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;

    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE;
    USART2->CR1 |= USART_CR1_IDLEIE;
    USART2->CR1 |= USART_CR1_UE;

    NVIC_SetPriority(USART2_IRQn, 2);
    NVIC_EnableIRQ(USART2_IRQn);

    NVIC_SetPriority(DMA1_Stream6_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

static void usart2_tx_dma_send(const uint8_t *data, uint32_t len) {
    if (len == 0 || len > TX_DMA_BUF_SZ) return;
    if (tx_busy) return;

    memcpy(tx_buf, data, len);

    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream6->CR & DMA_SxCR_EN) {}

    DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;

    DMA1_Stream6->NDTR = len;

    tx_busy = 1;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

static void tim7_telemetry_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    uint32_t psc = (TIM_APB1_HZ / 1000000U) - 1U; // 1 MHz
    uint32_t arr = (1000000U / TELEMETRY_HZ) - 1U;

    TIM7->CR1 = 0;
    TIM7->PSC = psc;
    TIM7->ARR = arr;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->SR = 0;
    TIM7->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM7_IRQn, 4);
    NVIC_EnableIRQ(TIM7_IRQn);

    TIM7->CR1 |= TIM_CR1_CEN;
}

static void send_help(void) {
    const char *s =
        "Commands:\n"
        "  pwm=0..100\n"
        "  dir=0|1\n"
        "  status\n"
        "  help\n";
    usart2_tx_dma_send((const uint8_t*)s, (uint32_t)strlen(s));
}

static void send_status(void) {
    char out[128];
    uint16_t a1 = adc_dma[0];
    uint16_t a2 = adc_dma[1];
    int n = snprintf(out, sizeof(out), "adc1=%u adc2=%u pwm=%u dir=%u\n", a1, a2, pwm_percent, dir_state);
    if (n > 0) usart2_tx_dma_send((const uint8_t*)out, (uint32_t)n);
}

static void handle_line(char *line) {
    while (*line && isspace((unsigned char)*line)) line++;
    size_t L = strlen(line);
    while (L && (line[L-1] == '\r' || line[L-1] == '\n' || isspace((unsigned char)line[L-1]))) {
        line[L-1] = 0;
        L--;
    }
    if (*line == 0) return;

    if (!strcasecmp(line, "help")) { send_help(); return; }
    if (!strcasecmp(line, "status")) { send_status(); return; }

    if (!strncasecmp(line, "pwm=", 4)) {
        long v = strtol(line + 4, NULL, 10);
        if (v < 0) v = 0;
        if (v > 100) v = 100;
        pwm_set_percent((uint8_t)v);
        const char *ok = "OK\n";
        usart2_tx_dma_send((const uint8_t*)ok, 3);
        return;
    }

    if (!strncasecmp(line, "dir=", 4)) {
        long v = strtol(line + 4, NULL, 10);
        dir_set((uint8_t)(v ? 1 : 0));
        const char *ok = "OK\n";
        usart2_tx_dma_send((const uint8_t*)ok, 3);
        return;
    }

    const char *err = "ERR\n";
    usart2_tx_dma_send((const uint8_t*)err, 4);
}

static void process_rx_dma(void) {
    uint32_t pos = RX_DMA_BUF_SZ - DMA1_Stream5->NDTR;
    if (pos == rx_last_pos) return;

    static char line[128];
    static uint32_t li = 0;

    while (rx_last_pos != pos) {
        uint8_t c = rx_dma[rx_last_pos];
        rx_last_pos++;
        if (rx_last_pos >= RX_DMA_BUF_SZ) rx_last_pos = 0;

        if (c == '\r') continue;

        if (c == '\n') {
            line[li] = 0;
            handle_line(line);
            li = 0;
            continue;
        }

        if (li < sizeof(line) - 1U) {
            line[li++] = (char)c;
        } else {
            li = 0;
        }
    }
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_IDLE) {
        volatile uint32_t tmp = USART2->DR;
        (void)tmp;
        process_rx_dma();
    }
    if (USART2->SR & USART_SR_ORE) {
        volatile uint32_t tmp = USART2->DR;
        (void)tmp;
    }
}

void DMA1_Stream6_IRQHandler(void) {
    if (DMA1->HISR & DMA_HISR_TCIF6) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        tx_busy = 0;
    }
    if (DMA1->HISR & DMA_HISR_TEIF6) {
        DMA1->HIFCR = DMA_HIFCR_CTEIF6;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        tx_busy = 0;
    }
}

void TIM7_IRQHandler(void) {
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;

        char out[64];
        uint16_t a1 = adc_dma[0];
        uint16_t a2 = adc_dma[1];
        int n = snprintf(out, sizeof(out), "%u %u %u %u\n", a1, a2, pwm_percent, dir_state);
        if (n > 0) usart2_tx_dma_send((const uint8_t*)out, (uint32_t)n);
    }
}

int main(void) {
    SCB->CPACR |= (3UL << (10U * 2U)) | (3UL << (11U * 2U));

    clock_180mhz_hse16();
    gpio_init();

    tim2_pwm_init();
    pwm_set_percent(0);
    dir_set(0);

    adc1_dma_init();
    usart2_dma_init();
    tim7_telemetry_init();

    const char *banner = "Lab5 v15 ready (UART 1M). Type help\n";
    usart2_tx_dma_send((const uint8_t*)banner, (uint32_t)strlen(banner));

    while (1) {
        __WFI();
    }
}
