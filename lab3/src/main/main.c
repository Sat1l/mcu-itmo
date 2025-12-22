#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>

#define UART_BAUD 38400U

#define N_SAMPLES 256U

#define VREF_V 3.3f

#define LED_PORT GPIOA
#define LED_PIN  5U

#define DAC_PORT GPIOA
#define DAC_PIN  4U

#define RXBUF_SIZE 64

static volatile uint16_t lut[N_SAMPLES];
static volatile uint32_t lut_idx = 0;

static volatile float target_freq_hz = 30.0f;
static volatile float target_amp_v   = 3.0f;

static volatile uint32_t led_div_reload = 1;
static volatile uint32_t led_div_counter = 0;
static const float led_toggle_hz = 2.0f;

static volatile char rxbuf[RXBUF_SIZE];
static volatile uint32_t rxlen = 0;
static volatile uint8_t cmd_ready = 0;

static void delay_cycles(volatile uint32_t n) {
    while (n--) { __NOP(); }
}

static void FPU_Enable(void) {
    SCB->CPACR |= (3UL << (10 * 2)) | (3UL << (11 * 2));
}

static void SystemClock_Config_180MHz(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    RCC->CFGR = 0;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= (16U << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (360U << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (0U << RCC_PLLCFGR_PLLP_Pos);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
    RCC->PLLCFGR |= (7U << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}

static void RCC_EnablePeripherals(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM6EN;
}

static void GPIO_Config(void) {
    LED_PORT->MODER &= ~(3U << (LED_PIN * 2));
    LED_PORT->MODER |=  (1U << (LED_PIN * 2));
    LED_PORT->OTYPER &= ~(1U << LED_PIN);
    LED_PORT->PUPDR  &= ~(3U << (LED_PIN * 2));

    GPIOA->MODER &= ~(3U << (DAC_PIN * 2));
    GPIOA->MODER |=  (3U << (DAC_PIN * 2));
    GPIOA->PUPDR &= ~(3U << (DAC_PIN * 2));

    GPIOA->MODER &= ~(0xFU << (2U * 2U));
    GPIOA->MODER |=  (0xAU << (2U * 2U));

    GPIOA->AFR[0] &= ~((0xFU << (2U * 4U)) | (0xFU << (3U * 4U)));
    GPIOA->AFR[0] |=  ((7U   << (2U * 4U)) | (7U   << (3U * 4U)));

    GPIOA->PUPDR &= ~(0xFU << (2U * 2U));
    GPIOA->PUPDR |=  (0x5U << (2U * 2U));
}

static void USART2_Config(uint32_t baud) {
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    uint32_t pclk1 = 45000000U;
    uint32_t usartdiv = (pclk1 + (baud / 2U)) / baud;
    USART2->BRR = usartdiv;

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART2->CR1 |= USART_CR1_UE;
}

static void USART2_SendByte(uint8_t b) {
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = b;
}

static void USART2_SendString(const char *s) {
    while (*s) USART2_SendByte((uint8_t)*s++);
}

static void DAC_Config(void) {
    DAC->CR = 0;
    DAC->CR |= DAC_CR_EN1;
    DAC->DHR12R1 = 2048U;
}

static void build_sine_lut(float amp_v, float vref_v) {
    float a = amp_v;
    if (a < 0.0f) a = 0.0f;
    if (a > vref_v) a = vref_v;

    float peak = (a / vref_v) * 2047.0f;
    float mid  = 2048.0f;

    for (uint32_t i = 0; i < N_SAMPLES; i++) {
        float x = (2.0f * 3.14159265358979323846f) * ((float)i / (float)N_SAMPLES);
        float y = mid + peak * sinf(x);
        if (y < 0.0f) y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;
        lut[i] = (uint16_t)(y + 0.5f);
    }
}

static uint32_t TIM6_GetClockHz(void) {
    return 90000000U;
}

static void TIM6_Config_FromFreq(float sine_hz) {
    if (sine_hz < 0.5f) sine_hz = 0.5f;
    if (sine_hz > 2000.0f) sine_hz = 2000.0f;

    float fs = sine_hz * (float)N_SAMPLES;
    uint32_t timclk = TIM6_GetClockHz();

    uint32_t best_psc = 0;
    uint32_t best_arr = 0;
    uint32_t best_err = 0xFFFFFFFFU;

    uint32_t target = (uint32_t)(fs + 0.5f);
    if (target < 1U) target = 1U;

    for (uint32_t psc = 0; psc <= 0xFFFFU; psc++) {
        uint32_t t = timclk / (psc + 1U);
        if (t < target) break;
        uint32_t arr = (t / target);
        if (arr == 0) arr = 1;
        if (arr > 0x10000U) continue;
        uint32_t real = t / arr;
        uint32_t err = (real > target) ? (real - target) : (target - real);
        if (err < best_err) {
            best_err = err;
            best_psc = psc;
            best_arr = arr - 1U;
            if (best_err == 0) break;
        }
    }

    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->PSC = best_psc;
    TIM6->ARR = best_arr;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR  = 0;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;

    float tim6_fs_hz = (float)timclk / ((float)(best_psc + 1U) * (float)(best_arr + 1U));
    float ticks_per_toggle = tim6_fs_hz / led_toggle_hz;
    if (ticks_per_toggle < 1.0f) ticks_per_toggle = 1.0f;
    led_div_reload = (uint32_t)(ticks_per_toggle + 0.5f);
    if (led_div_reload == 0) led_div_reload = 1;
    led_div_counter = 0;
}

static void NVIC_Enable_Interrupts(void) {
    NVIC_SetPriority(TIM6_DAC_IRQn, 2);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    NVIC_SetPriority(USART2_IRQn, 3);
    NVIC_EnableIRQ(USART2_IRQn);
}

static void trim_inplace(char *s) {
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);

    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[len - 1] = '\0';
        len--;
    }
}

static void process_line(char *line) {
    trim_inplace(line);
    if (line[0] == '\0') return;

    if ((line[0] == 'F' || line[0] == 'f') && line[1] == '=') {
        float f = strtof(&line[2], NULL);
        if (f > 0.5f && f < 2000.0f) {
            target_freq_hz = f;
            TIM6_Config_FromFreq(target_freq_hz);
            USART2_SendString("OK\r\n");
        } else {
            USART2_SendString("ERR\r\n");
        }
        return;
    }

    if ((line[0] == 'A' || line[0] == 'a') && line[1] == '=') {
        float a = strtof(&line[2], NULL);
        if (a >= 0.0f && a <= VREF_V) {
            target_amp_v = a;
            build_sine_lut(target_amp_v, VREF_V);
            USART2_SendString("OK\r\n");
        } else {
            USART2_SendString("ERR\r\n");
        }
        return;
    }

    if (strcasecmp(line, "status") == 0) {
        char out[80];
        int n = snprintf(out, sizeof(out), "F=%.3f Hz, A=%.3f V\r\n", (double)target_freq_hz, (double)target_amp_v);
        if (n > 0) USART2_SendString(out);
        return;
    }

    USART2_SendString("ERR\r\n");
}

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR &= ~TIM_SR_UIF;

        DAC->DHR12R1 = lut[lut_idx++];
        if (lut_idx >= N_SAMPLES) lut_idx = 0;

        if (++led_div_counter >= led_div_reload) {
            led_div_counter = 0;
            LED_PORT->ODR ^= (1U << LED_PIN);
        }
    }
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char c = (char)USART2->DR;

        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = (uint8_t)c;

        if (cmd_ready) return;

        if (c == '\r' || c == '\n') {
            if (rxlen < RXBUF_SIZE - 1U) {
                rxbuf[rxlen++] = '\n';
                cmd_ready = 1;
            } else {
                rxlen = 0;
                cmd_ready = 0;
            }
            return;
        }

        if (rxlen < RXBUF_SIZE - 1U) {
            rxbuf[rxlen++] = c;
        } else {
            rxlen = 0;
            cmd_ready = 0;
        }
    }

    if (USART2->SR & USART_SR_ORE) {
        volatile uint32_t tmp = USART2->DR;
        (void)tmp;
    }
}

int main(void) {
    FPU_Enable();
    SystemClock_Config_180MHz();

    RCC_EnablePeripherals();
    GPIO_Config();

    build_sine_lut(target_amp_v, VREF_V);

    USART2_Config(UART_BAUD);
    DAC_Config();

    NVIC_Enable_Interrupts();
    TIM6_Config_FromFreq(target_freq_hz);

    delay_cycles(2000000);

    USART2_SendString("Lab3 ready\r\n");
    USART2_SendString("F=<0.5..2000>  A=<0..3.3>  status\r\n");

    for (;;) {
        if (cmd_ready) {
            char line[RXBUF_SIZE];
            uint32_t n = rxlen;
            if (n >= RXBUF_SIZE) n = RXBUF_SIZE - 1U;

            for (uint32_t i = 0; i < n; i++) line[i] = (char)rxbuf[i];
            line[n] = '\0';

            rxlen = 0;
            cmd_ready = 0;

            process_line(line);
        }

        __WFI();
    }
}
