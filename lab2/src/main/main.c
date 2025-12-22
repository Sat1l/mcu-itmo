#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

uint8_t led_pins[8] = {4, 5, 6, 7, 8, 9, 10, 11};
char rx_buffer[64];
uint8_t rx_index = 0;
uint8_t led_state = 0;

#define PCLK1_HZ  16000000U
#define BAUDRATE  38400U

void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN;

    for (int i = 0; i < 8; i++) {
        GPIOC->MODER &= ~(0x3 << (led_pins[i] * 2));
        GPIOC->MODER |=  (0x1 << (led_pins[i] * 2));
    }

    GPIOA->MODER &= ~(0xF << (2 * 2));
    GPIOA->MODER |=  (0x2 << (2 * 2)) | (0x2 << (3 * 2));
    GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
    GPIOA->AFR[0] |=  ((7 << (2 * 4)) | (7 << (3 * 4)));
}

void usart2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = PCLK1_HZ / BAUDRATE;   // ≈ 104 для 4 МГц и 38400 бод
    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 0);
}

void clear_leds(void) {
    uint32_t mask = 0;
    for (int i = 0; i < 8; i++) mask |= (1 << led_pins[i]);
    GPIOC->BSRR = mask << 16;
    led_state = 0;
}

void set_all_leds(void) {
    uint32_t mask = 0;
    for (int i = 0; i < 8; i++) mask |= (1 << led_pins[i]);
    GPIOC->BSRR = mask;
    led_state = 0xFF;
}

void show_value(uint8_t value) {
    clear_leds();
    for (int i = 0; i < 8; i++) {
        if (value & (1 << i)) GPIOC->BSRR = 1 << led_pins[i];
    }
    led_state = value;
}

void USART2_SendByte(uint8_t data) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = data;
}

void USART2_SendString(const char *str) {
    while (*str) {
        USART2_SendByte(*str++);
    }
}

void ProcessCommand(char *cmd) {
    while (*cmd && isspace((unsigned char)*cmd)) cmd++;
    for (int i = strlen(cmd) - 1; i >= 0; i--) {
        if (isspace((unsigned char)cmd[i])) cmd[i] = '\0';
        else break;
    }

    if (*cmd == '\0') {
        return;
    }

    int value = 0;
    if (sscanf(cmd, "%d", &value) == 1 && value >= 0 && value <= 255) {
        show_value((uint8_t)value);
        USART2_SendString("OK\r\n");
    } else {
        USART2_SendString("Invalid number\r\n");
    }
}

void USART2_IRQHandler(void) {
    //проверяем RXNE
    if (USART2->SR & USART_SR_RXNE) {
        uint8_t byte = USART2->DR;

        if (byte == '\r') return;

        if (byte == '\n') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                ProcessCommand(rx_buffer);
                rx_index = 0;

                USART2_SendString("> ");
            } else {
                USART2_SendString("> ");
            }
        }
        else if (rx_index < sizeof(rx_buffer)-1) {
            rx_buffer[rx_index++] = byte;
        }
        else {
            rx_index = 0;
            USART2_SendString("\r\nBuffer overflow\r\n> ");
        }
    }

    if (USART2->SR & USART_SR_ORE) {
        volatile uint32_t temp = USART2->DR;
        (void)temp;
    }
}

int main(void) {
    gpio_init();
    usart2_init();
    clear_leds();

    for(volatile int i = 0; i < 100000; i++);

    USART2_SendString("STM32 Binary LED Display Ready\r\n");
    USART2_SendString("Send number 0-255, end with \\n\r\n");
    USART2_SendString("> ");

    while (1) {
        __WFI();
    }
}
