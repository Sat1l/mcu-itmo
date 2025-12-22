#include "stm32f446xx.h"
#include "stdint.h"
#include "config.h"

void SystemInit(void){
}

void SystemInitError(uint16_t error_source) {
	(void) error_source;
	while(1);
}
