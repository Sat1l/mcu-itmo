#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#define SYSCLK_HZ           180000000U
#define PCLK1_HZ            45000000U
#define TIM_APB1_HZ         90000000U

#define UART_BAUD           1000000U

#define DIR_PORT            GPIOB
#define DIR_PIN             5U          // PB5 = DIR1

#define PWM_PORT            GPIOB
#define PWM_PIN             4U          // PB4 = EN1 (PWM)

#define ENC_MOTOR_A_PORT    GPIOA
#define ENC_MOTOR_A_PIN     0U          // PA0 = A (motor encoder)
#define ENC_MOTOR_B_PORT    GPIOA
#define ENC_MOTOR_B_PIN     1U          // PA1 = B (motor encoder)

#define ENC_EXT_A_PORT      GPIOB
#define ENC_EXT_A_PIN       6U          // PB6 = A (external encoder)
#define ENC_EXT_B_PORT      GPIOB
#define ENC_EXT_B_PIN       7U          // PB7 = B (external encoder)

#define UART_TX_PORT        GPIOA
#define UART_TX_PIN         2U
#define UART_RX_PORT        GPIOA
#define UART_RX_PIN         3U

#define PWM_HZ              1000U
#define SPEED_HZ            100U
#define TELEMETRY_HZ        20U

#define RX_LINE_SZ          96

static volatile int32_t ext_zero = 0;

static volatile uint16_t motor_cnt_prev = 0;
static volatile int16_t  motor_delta = 0;

static volatile int16_t  target_pct = 0;     // -100..100 from external encoder
static volatile uint8_t  pwm_pct = 0;        // 0..100
static volatile uint8_t  dir_state = 0;      // 0/1

static volatile int16_t  kp = 2;             // P gain (tune)
static volatile uint16_t speed_scale = 200;  // desired motor delta per 10ms at 100%

static volatile uint8_t telemetry_flag = 0;

static volatile char rx_line[RX_LINE_SZ];
static volatile uint32_t rx_len = 0;
static volatile uint8_t cmd_ready = 0;

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

static void usart2_send_byte(uint8_t b) {
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = b;
}

static void usart2_send_str(const char *s) {
    while (*s) usart2_send_byte((uint8_t)*s++);
}

static void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // DIR PB5 output
    DIR_PORT->MODER &= ~(3U << (DIR_PIN * 2U));
    DIR_PORT->MODER |=  (1U << (DIR_PIN * 2U));
    DIR_PORT->OTYPER &= ~(1U << DIR_PIN);

    // PWM PB4 AF2 (TIM3_CH1)
    PWM_PORT->MODER &= ~(3U << (PWM_PIN * 2U));
    PWM_PORT->MODER |=  (2U << (PWM_PIN * 2U));
    PWM_PORT->AFR[0] &= ~(0xFU << (PWM_PIN * 4U));
    PWM_PORT->AFR[0] |=  (2U   << (PWM_PIN * 4U));

    // Motor encoder PA0/PA1 AF1 (TIM2_CH1/CH2)
    GPIOA->MODER &= ~(3U << (ENC_MOTOR_A_PIN * 2U));
    GPIOA->MODER &= ~(3U << (ENC_MOTOR_B_PIN * 2U));
    GPIOA->MODER |=  (2U << (ENC_MOTOR_A_PIN * 2U)) | (2U << (ENC_MOTOR_B_PIN * 2U));
    GPIOA->AFR[0] &= ~((0xFU << (ENC_MOTOR_A_PIN * 4U)) | (0xFU << (ENC_MOTOR_B_PIN * 4U)));
    GPIOA->AFR[0] |=  ((1U   << (ENC_MOTOR_A_PIN * 4U)) | (1U   << (ENC_MOTOR_B_PIN * 4U)));

    // External encoder PB6/PB7 AF2 (TIM4_CH1/CH2)
    GPIOB->MODER &= ~(3U << (ENC_EXT_A_PIN * 2U));
    GPIOB->MODER &= ~(3U << (ENC_EXT_B_PIN * 2U));
    GPIOB->MODER |=  (2U << (ENC_EXT_A_PIN * 2U)) | (2U << (ENC_EXT_B_PIN * 2U));
    GPIOB->AFR[0] &= ~((0xFU << (ENC_EXT_A_PIN * 4U)) | (0xFU << (ENC_EXT_B_PIN * 4U)));
    GPIOB->AFR[0] |=  ((2U   << (ENC_EXT_A_PIN * 4U)) | (2U   << (ENC_EXT_B_PIN * 4U)));

    // USART2 PA2/PA3 AF7
    UART_TX_PORT->MODER &= ~(3U << (UART_TX_PIN * 2U));
    UART_RX_PORT->MODER &= ~(3U << (UART_RX_PIN * 2U));
    UART_TX_PORT->MODER |=  (2U << (UART_TX_PIN * 2U)) | (2U << (UART_RX_PIN * 2U));
    GPIOA->AFR[0] &= ~((0xFU << (UART_TX_PIN * 4U)) | (0xFU << (UART_RX_PIN * 4U)));
    GPIOA->AFR[0] |=  ((7U   << (UART_TX_PIN * 4U)) | (7U   << (UART_RX_PIN * 4U)));
}

static void usart2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    USART2->BRR = (PCLK1_HZ + (UART_BAUD / 2U)) / UART_BAUD;

    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;
    USART2->CR1 |= USART_CR1_UE;

    NVIC_SetPriority(USART2_IRQn, 2);
    NVIC_EnableIRQ(USART2_IRQn);
}

static void tim3_pwm_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    uint32_t psc = (TIM_APB1_HZ / 1000000U) - 1U; // 1 MHz
    uint32_t arr = (1000000U / PWM_HZ) - 1U;      // 1 kHz

    TIM3->CR1 = 0;
    TIM3->PSC = psc;
    TIM3->ARR = arr;

    TIM3->CCMR1 = 0;
    TIM3->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

    TIM3->CCER = 0;
    TIM3->CCER |= TIM_CCER_CC1E;

    TIM3->CCR1 = 0;

    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->CR1 |= TIM_CR1_CEN;
}

static void pwm_set(uint8_t pct) {
    if (pct > 100U) pct = 100U;
    pwm_pct = pct;
    uint32_t arr = TIM3->ARR;
    uint32_t ccr = (pct * (arr + 1U)) / 100U;
    if (ccr > arr) ccr = arr;
    TIM3->CCR1 = ccr;
}

static void dir_set(uint8_t dir) {
    dir_state = (dir ? 1U : 0U);
    if (dir_state) DIR_PORT->BSRR = (1U << DIR_PIN);
    else           DIR_PORT->BSRR = (1U << (DIR_PIN + 16U));
}

static void tim2_encoder_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 = 0;
    TIM2->SMCR = 0;
    TIM2->CCMR1 = 0;
    TIM2->CCER = 0;

    TIM2->ARR = 0xFFFFU;

    TIM2->CCMR1 |= (1U << TIM_CCMR1_CC1S_Pos);
    TIM2->CCMR1 |= (1U << TIM_CCMR1_CC2S_Pos);

    TIM2->SMCR |= (3U << TIM_SMCR_SMS_Pos);

    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;

    motor_cnt_prev = 0;
}

static void tim4_encoder_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    TIM4->CR1 = 0;
    TIM4->SMCR = 0;
    TIM4->CCMR1 = 0;
    TIM4->CCER = 0;

    TIM4->ARR = 0xFFFFU;

    TIM4->CCMR1 |= (1U << TIM_CCMR1_CC1S_Pos);
    TIM4->CCMR1 |= (1U << TIM_CCMR1_CC2S_Pos);

    TIM4->SMCR |= (3U << TIM_SMCR_SMS_Pos);

    TIM4->CNT = 0;
    TIM4->CR1 |= TIM_CR1_CEN;

    ext_zero = 0;
}

static void tim5_speed_timer_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    uint32_t psc = (TIM_APB1_HZ / 10000U) - 1U; // 10 kHz
    uint32_t arr = (10000U / SPEED_HZ) - 1U;    // 100 Hz -> 10ms

    TIM5->CR1 = 0;
    TIM5->PSC = psc;
    TIM5->ARR = arr;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR = 0;
    TIM5->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM5_IRQn, 3);
    NVIC_EnableIRQ(TIM5_IRQn);

    TIM5->CR1 |= TIM_CR1_CEN;
}

static void tim7_telemetry_timer_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    uint32_t psc = (TIM_APB1_HZ / 10000U) - 1U; // 10 kHz
    uint32_t arr = (10000U / TELEMETRY_HZ) - 1U;

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

static int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

static void update_target_from_ext_encoder(void) {
    int32_t c = (int32_t)(uint16_t)TIM4->CNT;
    int32_t diff = c - ext_zero;

    if (diff >  32767) diff -= 65536;
    if (diff < -32768) diff += 65536;

    int32_t pct = diff / 4; // чувствительность (крутите внешний энкодер -> скорость)
    target_pct = clamp_i16(pct, -100, 100);
}

static void control_step_10ms(void) {
    update_target_from_ext_encoder();

    uint16_t m = (uint16_t)TIM2->CNT;
    motor_delta = (int16_t)(m - motor_cnt_prev);
    motor_cnt_prev = m;

    int16_t t = target_pct;
    if (t == 0) {
        pwm_set(0);
        return;
    }

    if (t > 0) dir_set(1); else dir_set(0);

    int32_t desired = ((int32_t)(t > 0 ? t : -t) * (int32_t)speed_scale) / 100;
    int32_t actual  = motor_delta;
    if (actual < 0) actual = -actual;

    int32_t err = desired - actual;
    int32_t upd = (int32_t)pwm_pct + (int32_t)kp * err / 10;

    if (upd < 0) upd = 0;
    if (upd > 100) upd = 100;

    pwm_set((uint8_t)upd);
}

static void send_help(void) {
    usart2_send_str(
        "help\n"
        "status\n"
        "zero\n"
        "kp=<int>\n"
        "scale=<int>\n"
    );
}

static void send_status(void) {
    char out[160];
    uint16_t m = (uint16_t)TIM2->CNT;
    uint16_t e = (uint16_t)TIM4->CNT;
    int n = snprintf(out, sizeof(out),
        "ext=%u zero=%ld target=%d pwm=%u dir=%u motor=%u d=%d kp=%d scale=%u\n",
        e, (long)ext_zero, (int)target_pct, (unsigned)pwm_pct, (unsigned)dir_state,
        m, (int)motor_delta, (int)kp, (unsigned)speed_scale
    );
    if (n > 0) usart2_send_str(out);
}

static void handle_line(char *s) {
    while (*s && isspace((unsigned char)*s)) s++;
    size_t L = strlen(s);
    while (L && (s[L-1] == '\r' || s[L-1] == '\n' || isspace((unsigned char)s[L-1]))) { s[L-1] = 0; L--; }
    if (*s == 0) return;

    if (!strcasecmp(s, "help")) { send_help(); return; }
    if (!strcasecmp(s, "status")) { send_status(); return; }
    if (!strcasecmp(s, "zero")) {
        ext_zero = (int32_t)(uint16_t)TIM4->CNT;
        target_pct = 0;
        pwm_set(0);
        send_status();
        return;
    }

    if (!strncasecmp(s, "kp=", 3)) {
        long v = strtol(s + 3, NULL, 10);
        if (v < 0) v = 0;
        if (v > 200) v = 200;
        kp = (int16_t)v;
        send_status();
        return;
    }

    if (!strncasecmp(s, "scale=", 6)) {
        long v = strtol(s + 6, NULL, 10);
        if (v < 1) v = 1;
        if (v > 2000) v = 2000;
        speed_scale = (uint16_t)v;
        send_status();
        return;
    }

    usart2_send_str("ERR\n");
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char c = (char)USART2->DR;

        if (cmd_ready) return;

        if (c == '\r') return;

        if (c == '\n') {
            if (rx_len < RX_LINE_SZ) rx_line[rx_len] = 0;
            cmd_ready = 1;
            return;
        }

        if (rx_len < RX_LINE_SZ - 1U) {
            rx_line[rx_len++] = c;
        } else {
            rx_len = 0;
        }
    }

    if (USART2->SR & USART_SR_ORE) {
        volatile uint32_t tmp = USART2->DR;
        (void)tmp;
    }
}

void TIM5_IRQHandler(void) {
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR &= ~TIM_SR_UIF;
        control_step_10ms();
    }
}

void TIM7_IRQHandler(void) {
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;
        telemetry_flag = 1;
    }
}

int main(void) {
    SCB->CPACR |= (3UL << (10U * 2U)) | (3UL << (11U * 2U));

    clock_180mhz_hse16();
    gpio_init();
    usart2_init();

    tim3_pwm_init();
    pwm_set(0);
    dir_set(0);

    tim2_encoder_init();
    tim4_encoder_init();

    ext_zero = (int32_t)(uint16_t)TIM4->CNT;

    tim5_speed_timer_init();
    tim7_telemetry_timer_init();

    usart2_send_str("Lab6 v15 ready. UART 1M. Type help\n");

    while (1) {
        if (cmd_ready) {
            char line[RX_LINE_SZ];
            uint32_t n = rx_len;
            if (n >= RX_LINE_SZ) n = RX_LINE_SZ - 1U;
            for (uint32_t i = 0; i < n; i++) line[i] = (char)rx_line[i];
            line[n] = 0;

            rx_len = 0;
            cmd_ready = 0;

            handle_line(line);
        }

        if (telemetry_flag) {
            telemetry_flag = 0;

            char out[128];
            uint16_t m = (uint16_t)TIM2->CNT;
            uint16_t e = (uint16_t)TIM4->CNT;
            int n = snprintf(out, sizeof(out), "%u %u %d %u %u\n",
                e, m, (int)motor_delta, (unsigned)pwm_pct, (unsigned)dir_state
            );
            if (n > 0) usart2_send_str(out);
        }

        __WFI();
    }
}
