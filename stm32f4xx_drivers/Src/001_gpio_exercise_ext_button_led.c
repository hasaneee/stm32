#include <stdio.h>
#include "stm32f446xx.h"

void delay(void){
	for(int i=0; i<=450000; i++);
}

int main(void){
	GPIO_Handle_t gpioInpt, gpioOutpt;
	gpioInpt.pGPIOx = GPIOA;
	gpioInpt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	gpioInpt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioInpt.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_SPEED_FAST;
	gpioInpt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	gpioInpt.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OP_TYPE_PP;

	gpioOutpt.pGPIOx = GPIOB;
	gpioOutpt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioOutpt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioOutpt.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_SPEED_FAST;
	gpioOutpt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioInpt);
	GPIO_Init(&gpioOutpt);

	uint8_t val;

	while(1){
		val = GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_5);
		if(val == 0){
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_9);
			delay();
		}
	}

	return 0;
}
