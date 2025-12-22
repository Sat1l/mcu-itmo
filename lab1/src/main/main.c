
#include <stdint.h>

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
} RCC_TypeDef;

#define RCC    ((RCC_TypeDef *)  0x40023800UL)
#define GPIOA  ((GPIO_TypeDef *) 0x40020000UL)
#define GPIOC  ((GPIO_TypeDef *) 0x40020800UL)
#define GPIOD  ((GPIO_TypeDef *) 0x40020C00UL)

#define SW1_PORT GPIOC
#define SW1_PIN  13u
#define SW2_PORT GPIOD
#define SW2_PIN  2u

#define LED_PORT GPIOC
static const uint8_t LED_FIRST_PIN = 4u;   /* PC4..PC11 */

#define BB_PERI_BASE   0x42000000UL
#define PERI_BASE      0x40000000UL
#define BB_PERI(addr,bit) (*(volatile uint32_t*)(BB_PERI_BASE + ((((uint32_t)(addr))-PERI_BASE)*32u) + ((bit)*4u)))

static void delay(volatile uint32_t d){ while(d--){} }

static inline uint32_t btn_read(GPIO_TypeDef* p, uint32_t pin){
    return (p->IDR >> pin) & 1u;
}

static void leds_show_byte(uint8_t v){
    for(uint32_t i=0;i<8;i++){
        uint32_t pin = LED_FIRST_PIN + i;
        BB_PERI(&LED_PORT->ODR, pin) = (v >> i) & 1u;
    }
}

static void leds_off(void){
    for(uint32_t i=0;i<8;i++){
        uint32_t pin = LED_FIRST_PIN + i;
        BB_PERI(&LED_PORT->ODR, pin) = 0u;
    }
}

static uint8_t xorshift8(uint8_t x){
    x ^= x << 3;
    x ^= x >> 5;
    x ^= x << 1;
    return x ? x : 0xA7;
}

int main(void){
    RCC->AHB1ENR |= (1u<<0) | (1u<<2) | (1u<<3);
    (void)RCC->AHB1ENR;

    for(uint32_t p=LED_FIRST_PIN;p<=11u;p++){
        LED_PORT->MODER   = (LED_PORT->MODER & ~(3u << (p*2))) | (1u << (p*2));
        LED_PORT->OTYPER &= ~(1u << p);
        LED_PORT->OSPEEDR = (LED_PORT->OSPEEDR & ~(3u << (p*2))) | (2u << (p*2));
        LED_PORT->PUPDR  &= ~(3u << (p*2));
    }

    SW1_PORT->MODER  &= ~(3u << (SW1_PIN*2));
    SW1_PORT->PUPDR   = (SW1_PORT->PUPDR & ~(3u << (SW1_PIN*2))) | (1u << (SW1_PIN*2));

    SW2_PORT->MODER  &= ~(3u << (SW2_PIN*2));
    SW2_PORT->PUPDR   = (SW2_PORT->PUPDR & ~(3u << (SW2_PIN*2))) | (1u << (SW2_PIN*2));

    leds_off();

    uint8_t rnd = 0x5D;
    uint32_t sw1_last = btn_read(SW1_PORT, SW1_PIN);
    uint32_t sw2_last = btn_read(SW2_PORT, SW2_PIN);

    for(;;){
        uint32_t s1 = btn_read(SW1_PORT, SW1_PIN);
        uint32_t s2 = btn_read(SW2_PORT, SW2_PIN);

        if (sw1_last && !s1){
            delay(50000);
            if (btn_read(SW1_PORT, SW1_PIN) == 0u){
                rnd = xorshift8(rnd);
                leds_show_byte(rnd);
                while(btn_read(SW1_PORT, SW1_PIN) == 0u){}
                delay(50000);
            }
        }

        if (sw2_last && !s2){
            delay(50000);
            if (btn_read(SW2_PORT, SW2_PIN) == 0u){
                leds_off();
                while(btn_read(SW2_PORT, SW2_PIN) == 0u){}
                delay(50000);
            }
        }

        sw1_last = s1;
        sw2_last = s2;
    }
}
