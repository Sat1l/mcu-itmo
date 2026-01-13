#include "stm32f103xb.h"
#include "config.h"

// sets 72 MHz clock frequency
void clock_config(void){
	RCC->CR |= RCC_CR_HSION; // Internal high-speed clock enable
	while (!(RCC->CR & RCC_CR_HSIRDY)); // wait until HSI is ready

	RCC->CR |= RCC_CR_HSEON; // HSE clock enable (external quartz 8 Mhz)
	while (!(RCC->CR & RCC_CR_HSERDY)); // wait until HSE is ready

	RCC->CR |= RCC_CR_CSSON; // enable clock security system

	RCC->CFGR |= RCC_CFGR_SW_HSE; // select HSE as system clock

	FLASH->ACR |= (0b010 << FLASH_ACR_LATENCY_Pos); //010 Two wait states, if 48 MHz < SYSCLK <= 72 MHz
	FLASH->ACR |= FLASH_ACR_PRFTBE; // enable prefetch buffer 

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB prescaler = 1 (AHB clock = 72 MHz) 

	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // APB1 prescaler = 4 (APB1 clock = 18 MHz) 
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // APB2 prescaler = 2 (APB2 clock = 36 MHz) 

	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; // ADC prescaler = 6 (ADC clock = 12 MHz) 

	// PLL 
	RCC->CFGR |= RCC_CFGR_PLLSRC; // PLL source = HSE 
	RCC->CFGR &= ~RCC_CFGR_PLLXTPRE_HSE; // HSE clock not divided 

	RCC->CFGR |= RCC_CFGR_PLLMULL9; // PLL multiplication factor = 9 (8 MHz * 9 = 72 MHz) 

	RCC->CFGR &= ~RCC_CFGR_USBPRE; // USB 72MHz/1.5 = 48MHz

	RCC->CFGR |= RCC_CFGR_MCO_PLLCLK_DIV2; // MCO = PLL clock divided by 2 (36 MHz) 

	RCC->CR |= RCC_CR_PLLON; // PLL enables
	while (!(RCC->CR & RCC_CR_PLLRDY)); // wait until PLL is ready

	RCC->CFGR &= ~RCC_CFGR_SW; // clear SW bits
	RCC->CFGR |= RCC_CFGR_SW_PLL; // select PLL as system clock 

	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL; // select PLL as system clock
}

volatile uint32_t millis = 0; // milliseconds counter

void enable_systick(void) {
	SysTick->LOAD = AHB_FREQ / 1000 - 1; // 1ms tick
	SysTick->VAL = 0; // clear the counter
	// enable the timer with interrupt
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // use processor clock
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // enable interrupt

	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // enable the timer
}

void SysTick_Handler(){
	millis++; // increment milliseconds counter
}

void SystemInit(void){
	clock_config();
	enable_systick();
}



void SystemInitError(uint16_t error_source) {
	(void) error_source;
	while(1);
}
