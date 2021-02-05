/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jan 5, 2021
 *      Author: HASAN
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_


#include "stm32f446xx.h"
#include "stdint.h"

/*
 * This is Handle structure of GPIO pin
 */

typedef struct {
	uint8_t GPIO_PinNumber;           /* find possible value from @GPIO PIN NUMBERS */
	uint8_t GPIO_PinMode;             /* find possible value from @GPIO PIN MODES */
	uint8_t GPIO_PinSpeed;            /* find possible value from @GPIO PIN SPEED */
	uint8_t GPIO_PinPuPdControl;      /* find possible value from @GPIO PIN PULL-UP AND PULL-DOWN */
	uint8_t GPIO_PinOPtype;           /* find possible value from @GPIO PIN OUTPUT TYPE */
	uint8_t GPIO_PinAltFunMode;       /* find possible value from @GPIO ALT FUN MODE */
} GPIO_PinConfig_t;


/*
 * This is Handle structure of GPIO pin
 */

typedef struct {
	GPIO_Regdef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;


/*
 * @GPIO PIN NUMBERS
 */

#define GPIO_PIN_NO_0                0
#define GPIO_PIN_NO_1                1
#define GPIO_PIN_NO_2                2
#define GPIO_PIN_NO_3                3
#define GPIO_PIN_NO_4                4
#define GPIO_PIN_NO_5                5
#define GPIO_PIN_NO_6                6
#define GPIO_PIN_NO_7                7
#define GPIO_PIN_NO_8                8
#define GPIO_PIN_NO_9                9
#define GPIO_PIN_NO_10               10
#define GPIO_PIN_NO_11               11
#define GPIO_PIN_NO_12               12
#define GPIO_PIN_NO_13               13
#define GPIO_PIN_NO_14               14
#define GPIO_PIN_NO_15               15


/*
 * @GPIO PIN MODES
 */

#define GPIO_MODE_IN                 0
#define GPIO_MODE_OUT                1
#define GPIO_MODE_ALT_FUN            2
#define GPIO_MODE_ANALOG             3
#define GPIO_MODE_IT_FT              4
#define GPIO_MODE_IT_RT              5
#define GPIO_MODE_IT_RFT             6


/*
 * @GPIO PIN SPEED
 */

#define GPIO_PIN_SPEED_LOW           0
#define GPIO_PIN_SPEED_MEDIUM        1
#define GPIO_PIN_SPEED_FAST          2
#define GPIO_PIN_SPEED_HIGH          3


/*
 * @GPIO PIN PULL-UP AND PULL-DOWN
 */

#define GPIO_PIN_NO_PUPD               0
#define GPIO_PIN_PU                    1
#define GPIO_PIN_PD                    2


/*
 * @GPIO PIN OUTPUT TYPE
 */

#define GPIO_OP_TYPE_PP                0
#define GPIO_OP_TYPE_OD                1


/*
 * @GPIO ALT FUN MODE
 */


/********************************************************************
 *               APIs supports by the driver
 *    For more Information check the function definition
 *********************************************************************/

/*
 *  GPIO peripheral clock setup;
 */

void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx, uint8_t ENorDI);


/*
 *  GPIO Init and DeInit;
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Regdef_t *pGPIOx);


/*
 *  GPIO Data read and write;
 */

uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_Regdef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber);

/*
 *  IRQ Configuration and ISR Handling;
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_ITQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
