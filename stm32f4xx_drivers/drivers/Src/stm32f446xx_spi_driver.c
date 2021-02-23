/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Feb 14, 2021
 *  Author: HASAN
 */

#include "stm32f446xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 ***********************************************************************/

void SPI_PeriClockControl(SPI_Regdef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		} else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		} else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	} else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		} else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		} else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 ***********************************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle){

	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register
	uint32_t tempReg = 0;

	//1. Configure SPI device mode
	tempReg |= (pSPIHandle->GPIO_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//2. Configure SPI Bus Configuration
	if(pSPIHandle->GPIO_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidi mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->GPIO_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//bidi mode should be set
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->GPIO_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//BIDI mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempReg |= (1 << SPI_CR1_RXONLY);
	}

	//Configure SPI Clock Speed
	tempReg |= (pSPIHandle->GPIO_PinConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//Configure SPI Data Frame Format
	tempReg |= (pSPIHandle->GPIO_PinConfig.SPI_DFF << SPI_CR1_DFF);

	//Configure SPI Clock Polarity
	tempReg |= (pSPIHandle->GPIO_PinConfig.SPI_CPOL << SPI_CR1_CPOL);

	//Configure SPI Clock Phase
	tempReg |= (pSPIHandle->GPIO_PinConfig.SPI_CPHA << SPI_CR1_CPHA);

	//Configure SPI Software Slave Management
	tempReg |= (pSPIHandle->GPIO_PinConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempReg;

}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  This is blocking call
 ***********************************************************************/

void SPI_PeripheralControl(SPI_Regdef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIconfig
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  This is blocking call
 ***********************************************************************/
void SPI_SSIconfig(SPI_Regdef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEconfig
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  This is blocking call
 ***********************************************************************/
void SPI_SSOEconfig(SPI_Regdef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 ***********************************************************************/
uint8_t SPI_GetFlagStatus(SPI_Regdef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  This is blocking call
 ***********************************************************************/

void SPI_SendData(SPI_Regdef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){
	if(len > 0){
		// 1.check TXE status
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2.check DFF register
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			// 16-bit DFF
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			len--;
			len--;
			(uint16_t *)pTxBuffer++;
		} else{
			// 8-bit DFF
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  This is blocking call
 ***********************************************************************/
void SPI_ReceiveData(SPI_Regdef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){
	if(len > 0){
		// 1.check RXNE status
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2.check DFF register
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			// 16-bit DFF
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			len--;
			len--;
			(uint16_t *)pRxBuffer++;
		} else{
			// 8-bit DFF
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}
