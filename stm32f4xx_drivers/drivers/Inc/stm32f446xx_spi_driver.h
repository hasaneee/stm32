/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Feb 14, 2021
 *      Author: HASAN
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"
#include "stdint.h"

/*
 * This is Pin Configuration structure of SPI peripheral
 */

typedef struct {
	uint8_t SPI_DeviceMode;           /* find possible value from @GPIO PIN NUMBERS */
	uint8_t SPI_BusConfig;             /* find possible value from @GPIO PIN MODES */
	uint8_t SPI_SclkSpeed;            /* find possible value from @GPIO PIN SPEED */
	uint8_t SPI_DFF;      /* find possible value from @GPIO PIN PULL-UP AND PULL-DOWN */
	uint8_t SPI_CPOL;           /* find possible value from @GPIO PIN OUTPUT TYPE */
	uint8_t SPI_CPHA;       /* find possible value from @GPIO ALT FUN MODE */
	uint8_t SPI_SSM;
} SPI_PinConfig_t;


/*
 * This is Handle structure of SPI peripheral
 */

typedef struct {
	SPI_Regdef_t *pSPIx;
	SPI_PinConfig_t GPIO_PinConfig;
} SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER    1
#define SPI_DEVICE_MODE_SLAVE     0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD                1
#define SPI_BUS_CONFIG_HD                2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2             	0
#define SPI_SCLK_SPEED_DIV4             	1
#define SPI_SCLK_SPEED_DIV8             	2
#define SPI_SCLK_SPEED_DIV16             	3
#define SPI_SCLK_SPEED_DIV32             	4
#define SPI_SCLK_SPEED_DIV64             	5
#define SPI_SCLK_SPEED_DIV128             	6
#define SPI_SCLK_SPEED_DIV256             	7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS 	0
#define SPI_DFF_16BITS  1

/*
 * @CPOL
 */
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN     1
#define SPI_SSM_DI     0

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)


/********************************************************************
 *               APIs supports by the driver
 *    For more Information check the function definition
 *********************************************************************/

/*
 *  SPI peripheral clock setup;
 */

void SPI_PeriClockControl(SPI_Regdef_t *pSPIx, uint8_t ENorDI);


/*
 *  SPI Init and DeInit;
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Regdef_t *pSPIx);


/*
 *  Data Send and Receive;
 */

void SPI_SendData(SPI_Regdef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_Regdef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);


/*
 *  IRQ Configuration and ISR Handling;
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_ITQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 *  SPI Peripheral control API;
 */
uint8_t SPI_GetFlagStatus(SPI_Regdef_t *pSPIx , uint32_t FlagName);

void SPI_PeripheralControl(SPI_Regdef_t *pSPIx, uint8_t ENorDI);

void SPI_SSIconfig(SPI_Regdef_t *pSPIx, uint8_t ENorDI);

void SPI_SSOEconfig(SPI_Regdef_t *pSPIx, uint8_t ENorDI);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
