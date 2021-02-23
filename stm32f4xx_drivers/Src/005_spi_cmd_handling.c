#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"

//extern void initialise_monitor_handles();

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9


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

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if(ackbyte == (uint8_t)0xF5)
	{
		//ack
		return 1;
	}

	return 0;
}

int main(void){

	//char user_data[] = "Hello World";

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	//initialise_monitor_handles();

	printf("Application is running\n");

	SPI2_gpioInit();

	SPI2_Init();

	GPIO_userButtonInit();

	printf("SPI Init. done\n");

	//SPI_SSIconfig(SPI2, ENABLE); //SSI not use because SSM is disable
	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEconfig(SPI2, ENABLE);

	while(1){
		// 1. COMMAND_LED_CTRL
		//stay hang in while loop while MCU user button is not pressed
		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);


		//Send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//send arguments
			SPI_SendData(SPI2,args,2);

			// dummy read
			SPI_ReceiveData(SPI2,args,2);
			printf("COMMAND_LED_CTRL Executed\n");
		}
		//end of COMMAND_LED_CTRL

		// 2.COMMAND_SENSOR_READ
		//stay hang in while loop while MCU user button is not pressed
		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2,args,1);

			// dummy read
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}
		//end of COMMAND_SENSOR_READ

		// 3. COMMAND_LED_READ
		// wait until pill is high
		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay(); // delay for 250ms

		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			uint8_t pin_status;
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2,args,1);

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			SPI_ReceiveData(SPI2,&pin_status,1);
			printf("Arduino Pin ststus %d\n",pin_status);
		}
		//end of COMMAND_LED_READ

		//4. COMMAND_PRINT
		// wait until pill is high
		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay(); // delay for 250ms

		commandcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t message[] = "Hello ! How are you ??";

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI2,args,1);

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			for(int i=0; i<args[0]; i++){
				//send arguments
				SPI_SendData(SPI2,&message[i],1);

				// do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);
			}
			printf("COMMAND_PRINT Executed \n");
		}
		//end of COMMAND_PRINT

		//5. CMD_ID_READ
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}

		// Lets confirm SPI2 is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}



	return 0;
}
