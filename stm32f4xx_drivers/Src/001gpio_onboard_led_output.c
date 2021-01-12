#include <stdio.h>
#include "stm32f446xx.h"

void delay(void){
	for(uint32_t i=0; i<=500000; i++);
}

int main(){
	/*
		GPIO_Handle_t gpioOutput;
		gpioOutput.pGPIOx = GPIOA;
		gpioOutput.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
		gpioOutput.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		gpioOutput.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_SPEED_FAST;
		gpioOutput.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
		gpioOutput.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OP_TYPE_PP;
	*/

	GPIO_Handle_t gpioUserIn;
	gpioUserIn.pGPIOx = GPIOC;
	gpioUserIn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioUserIn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioUserIn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_SPEED_FAST;
	gpioUserIn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_Handle_t gpioOutput1;
	gpioOutput1.pGPIOx = GPIOA;
	gpioOutput1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioOutput1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioOutput1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_SPEED_FAST;
	gpioOutput1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	gpioOutput1.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OP_TYPE_OD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioOutput1);
	GPIO_Init(&gpioUserIn);

	uint8_t val;

	while(1){
		val = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13);
		if(val == 0){
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
			delay();
		}
	}



	return 0;
}
