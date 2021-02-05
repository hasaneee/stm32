#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"

void delay(void){
	// 200ms delay when system clock in 16Mz
	for(uint32_t i=0; i<=500000; i++);
}

int main(void){
	GPIO_Handle_t GpioInput, GpioLed;
	memset(&GpioInput, 0, sizeof(GpioInput));
	memset(&GpioLed, 0, sizeof(GpioLed));

	GpioInput.pGPIOx = GPIOB;
	GpioInput.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioInput.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GpioInput.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_SPEED_MEDIUM;
	GpioInput.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_SPEED_MEDIUM;
	GpioLed.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioInput);
	GPIO_Init(&GpioLed);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_ITQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);

	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_4, 1);
	delay();
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_4, 0);
}
