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

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

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
	//SPI2pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPI2pins);

	//SPI2_MOSI Pin
	SPI2pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI2pins);
}

void SPI2_Init(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.GPIO_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.GPIO_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.GPIO_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // 8MHz clock speed
	SPI2handle.GPIO_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.GPIO_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.GPIO_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.GPIO_PinConfig.SPI_SSM = SPI_SSM_DI; //Software slave management Disable

	SPI_Init(&SPI2handle);
}

void GPIO_userButtonInit(void){
	GPIO_Handle_t GpioInput;
	memset(&GpioInput, 0, sizeof(GpioInput));

	GpioInput.pGPIOx = GPIOC;
	GpioInput.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioInput.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioInput.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_SPEED_MEDIUM;
	GpioInput.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_Init(&GpioInput);
}

int main(void){

	char user_data[] = "Hello World";

	SPI2_gpioInit();

	SPI2_Init();

	GPIO_userButtonInit();

	//SPI_SSIconfig(SPI2, ENABLE); //SSI not use because SSM is disable
	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEconfig(SPI2, ENABLE);

	while(1){
		//stay hang in while loop while MCU user button is not pressed
		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// frist send lenght information
		uint8_t data_len = strlen(user_data);
		SPI_SendData(SPI2, &data_len, 1);

		// Send data via MOSI pin of SPI2 peripheral
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// Lets confirm SPI2 is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}



	return 0;
}
