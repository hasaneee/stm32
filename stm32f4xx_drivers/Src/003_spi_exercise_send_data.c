#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"

/*
 * SPI2 PINS:
 * PB9 --> SPI2_NSS
 * PB10 --> SPI2_SCK
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * ALT Function Mode: 5
 */

void SPI2_gpioInit(void){
	GPIO_Handle_t SPI2pins;
	SPI2pins.pGPIOx = GPIOB;
	SPI2pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUN;
	SPI2pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI2pins.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OP_TYPE_PP;
	SPI2pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	SPI2pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_SPEED_FAST;

	//SPI2_CLK Pin
	SPI2pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&SPI2pins);

	//SPI2_NSS Pin
	SPI2pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&SPI2pins);

	//SPI2_MISO Pin
	SPI2pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPI2pins);

	//SPI2_MOSI Pin
	SPI2pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI2pins);
}

void SPI2_Init(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.GPIO_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.GPIO_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.GPIO_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // 8MHz clock speed
	SPI2handle.GPIO_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.GPIO_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.GPIO_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.GPIO_PinConfig.SPI_SSM = SPI_SSM_EN; //Software slave management ENABLE

	SPI_Init(&SPI2handle);
}

int main(void){

	char user_data[] = "Hello World";

	SPI2_gpioInit();

	SPI2_Init();

	SPI_SSIconfig(SPI2, ENABLE);

	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}

