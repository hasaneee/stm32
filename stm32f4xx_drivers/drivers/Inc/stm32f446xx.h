/*
 * stm32f446xx.h
 *
 *  Created on: Jan 3, 2021
 *      Author: HASAN
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include "stdint.h"

#define __vo volatile

/****************** Processor Specific details *******************/

/*
 * ARM CoetexMX processor NVIC_ISERx register addresses
 */

#define NVIC_ISER0             (__vo uint32_t*)0xE000E100
#define NVIC_ISER1             (__vo uint32_t*)0xE000E104
#define NVIC_ISER2             (__vo uint32_t*)0xE000E108
#define NVIC_ISER3             (__vo uint32_t*)0xE000E10C


/*
 * ARM CoetexMX processor NVIC_ICERx register addresses
 */

#define NVIC_ICER0             (__vo uint32_t*)0XE000E180
#define NVIC_ICER1             (__vo uint32_t*)0xE000E104
#define NVIC_ICER2             (__vo uint32_t*)0xE000E108
#define NVIC_ICER3             (__vo uint32_t*)0xE000E10C


/*
 * ARM CoetexMX processor NVIC_IPRx register base addresses
 */

#define NVIC_IPR               (__vo uint32_t*)0xE000E400
#define NO_PR_BITS_IMPLEMENTED 4



/*
 * Base address of FLASH and SRAM memories
 */

#define FLASH_BASEADDR          0x08000000U
#define SRAM1_BASEADDR          0x20000000U
#define SRAM2_BASEADDR          0x2001C000U
#define SRAM_BASEADDR           SRAM1_BASEADDR
#define ROM_BASEADDR            0x1FFF0000U


/*
 * AHBx and APBx bus peripheral base address
 */

#define PERIPH_BASEADDR         0x40000000U
#define AHB1PERIPH_BASEADDR     0x40020000U
#define AHB2PERIPH_BASEADDR     0x50000000U
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR     0x40010000U

/*
 * Base address of peripheral which are hanging on AHB1 bus
 */

#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3800)
#define GPIOA_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1C00)


/*
 * Base address of peripheral which are hanging on APB1 bus
 */

#define I2C1_BASEADDR            (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR            (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR            (APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR            (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR            (APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR          (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR          (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR           (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR           (APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base address of peripheral which are hanging on APB2 bus
 */

#define SPI1_BASEADDR            (APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR          (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR          (APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR            (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR          (APB2PERIPH_BASEADDR + 0x3800)

/*
 * peripheral Register definition structure
 */

typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
} RCC_Regdef_t;

typedef struct {
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDER;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
} GPIO_Regdef_t;

typedef struct {
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_Regdef_t;

typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	     uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	     uint32_t RESERVED2[2];
	__vo uint32_t CFGR;
} SYSCFG_Regdef_t;



/*
 * peripheral definition (peripheral base address typecasted to xxx_Regdef_t)
 */

#define GPIOA                    ((GPIO_Regdef_t*)GPIOA_BASEADDR)
#define GPIOB                    ((GPIO_Regdef_t*)GPIOB_BASEADDR)
#define GPIOC                    ((GPIO_Regdef_t*)GPIOC_BASEADDR)
#define GPIOD                    ((GPIO_Regdef_t*)GPIOD_BASEADDR)
#define GPIOE                    ((GPIO_Regdef_t*)GPIOE_BASEADDR)
#define GPIOF                    ((GPIO_Regdef_t*)GPIOF_BASEADDR)
#define GPIOG                    ((GPIO_Regdef_t*)GPIOG_BASEADDR)
#define GPIOH                    ((GPIO_Regdef_t*)GPIOH_BASEADDR)

#define RCC                      ((RCC_Regdef_t*)RCC_BASEADDR)
#define EXTI                     ((EXTI_Regdef_t*)EXTI_BASEADDR)
#define SYSCFG                   ((SYSCFG_Regdef_t*)SYSCFG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()          (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()          (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()          (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()          (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()          (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()          (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()          (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()          (RCC->AHB1ENR |= (1 << 7))


/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()          (RCC->APB2ENR |= (1 << 13))

/*
 * Clock Enable Macros for USARTx and UARTx peripherals
 */

#define USART1_PCLK_EN()          (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()          (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()          (RCC->APB2ENR |= (1 << 18))
#define UART4_PCLK_EN()           (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()           (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()          (RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()          (RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()          (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()          (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()          (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()          (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()          (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()          (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()          (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()          (RCC->AHB1ENR &= ~(1 << 7))


/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 23))


/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 13))


/*
 * Clock Disable Macros for USARTx and UARTx peripherals
 */

#define USART1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 18))
#define UART4_PCLK_DI()           (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()           (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripherals
*/

#define SYSCFG_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 14))


/*
 * GPIO registers reset macros
*/

#define GPIOA_REG_RESET()         do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()         do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()         do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()         do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()         do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()         do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()         do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()         do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)


/*
 * Return portcoode for given gpiox
 */

#define GPIO_BASE_ADDR_TO_CODE(x) ((x == GPIOA)?0:\
		                          (x == GPIOB)?1:\
				                  (x == GPIOC)?2:\
						          (x == GPIOD)?3:\
								  (x == GPIOE)?4:\
							      (x == GPIOF)?5:\
								  (x == GPIOG)?6:0 )


/*
 * IRQ (Interrupt Request) Numbers of STM32F446xx MCU
 */

#define IRQ_NO_EXTIO              6
#define IRQ_NO_EXTI1              7
#define IRQ_NO_EXTI2              8
#define IRQ_NO_EXTI3              9
#define IRQ_NO_EXTI4              10
#define IRQ_NO_EXTI9_5            23
#define IRQ_NO_EXTI15_10          40


/*
 * Macros for all the posible priority levels
 */

#define NVIC_IRQ_PRI0             0
#define NVIC_IRQ_PRI15            15


/*
 * Generic Macros
 */

#define ENABLE                   1
#define DISABLE                  0
#define SET                      ENABLE
#define RESET                    DISABLE

#include "stm32f446xx_gpio_driver.h"

#endif /* INC_STM32F446XX_H_ */
