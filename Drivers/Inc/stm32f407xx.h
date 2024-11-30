/*
 *	stm32f407xx.h
 *
 *	Created: Sep 5, 2024
 *  Author : @GHP
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/******************************************************************************
 ********************************GENERIC MACROS********************************
 ******************************************************************************/
#define ENABLE 				1
#define DISABLE 			0
#define SET 				1
#define RESET 				0

/******************************************************************************
 ********************************BASE ADDRESSES********************************
 ******************************************************************************/

/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * base addresses of Flash, SRAM and System memories
 */
#define FLASH_ADDR		0x08000000U
#define ROM_ADDR		0x1FFF0000U
#define SRAM1_ADDR		0x20000000U
#define SRAM2_ADDR		0x2001C000U

/*
 * base addresses of APBx and AHBx Bus Peripheral
 */
#define PERIPH_ADDR			0x40000000U
#define APB1_PERIPH_ADDR	0x40000000U
#define APB2_PERIPH_ADDR	0x40010000U
#define AHB1_PERIPH_ADDR	0x40020000U
#define AHB2_PERIPH_ADDR	0x50000000U

/*
 * base addresses of peripherals hanging on APB1 bus
 */
#define SPI2_ADDR		(APB1_PERIPH_ADDR + 0x3800)
#define SPI3_ADDR		(APB1_PERIPH_ADDR + 0x3C00)
#define USART2_ADDR		(APB1_PERIPH_ADDR + 0x4400)
#define USART3_ADDR		(APB1_PERIPH_ADDR + 0x4800)
#define UART4_ADDR		(APB1_PERIPH_ADDR + 0x4C00)
#define UART5_ADDR		(APB1_PERIPH_ADDR + 0x5000)
#define I2C1_ADDR		(APB1_PERIPH_ADDR + 0x5400)
#define I2C2_ADDR		(APB1_PERIPH_ADDR + 0x5800)
#define I2C3_ADDR		(APB1_PERIPH_ADDR + 0x5C00)

/*
 * base addresses of peripherals hanging on (APB2 bus
 */
#define USART1_ADDR		(APB2_PERIPH_ADDR + 0x1000)
#define USART6_ADDR		(APB2_PERIPH_ADDR + 0x1400)
#define SPI1_ADDR		(APB2_PERIPH_ADDR + 0x3000)
#define SYSCFG_ADDR		(APB2_PERIPH_ADDR + 0x3800)
#define EXTI_ADDR		(APB2_PERIPH_ADDR + 0x3C00)

/*
 * base addresses of peripherals hanging on (AHB1 bus
 */
#define GPIOA_ADDR		(AHB1_PERIPH_ADDR + 0x0000)
#define GPIOB_ADDR		(AHB1_PERIPH_ADDR + 0x0400)
#define GPIOC_ADDR		(AHB1_PERIPH_ADDR + 0x0800)
#define GPIOD_ADDR		(AHB1_PERIPH_ADDR + 0x0C00)
#define GPIOE_ADDR		(AHB1_PERIPH_ADDR + 0x1000)
#define GPIOF_ADDR		(AHB1_PERIPH_ADDR + 0x1400)
#define GPIOG_ADDR		(AHB1_PERIPH_ADDR + 0x1800)
#define GPIOH_ADDR		(AHB1_PERIPH_ADDR + 0x1C00)
#define GPIOI_ADDR		(AHB1_PERIPH_ADDR + 0x2000)
#define CRC_ADDR		(AHB1_PERIPH_ADDR + 0x3000)
#define RCC_ADDR		(AHB1_PERIPH_ADDR + 0x3800)

/******************************************************************************
 ******************PERIPHERAL REGISTER DEFINITION STRUCTURES ******************
 ******************************************************************************/

/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;			// port mode
	__vo uint32_t OTYPER;			// port output type
	__vo uint32_t OSPEEDR;			// port output speed
	__vo uint32_t PUPDR;			// port pull-up or pull-down
	__vo uint32_t IDR;				// port input data
	__vo uint32_t ODR;				// port output data
	__vo uint32_t BSSR;				// port bit set / reset
	__vo uint32_t LCKR;				// port configuration lock
	__vo uint32_t AFR[2];			// alternate function
} GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;					// clock control
  __vo uint32_t PLLCFGR;			// PLL clock outputs according to the formulas
  __vo uint32_t CFGR; 				// clock configuration
  __vo uint32_t CIR;  				// clock interrupt
  __vo uint32_t AHB1RSTR;			// AHB1 peripheral reset
  __vo uint32_t AHB2RSTR;			// AHB2 peripheral reset
  __vo uint32_t AHB3RSTR;			// AHB3 peripheral reset
  uint32_t      RESERVED0;     		// reserved                                                      */
  __vo uint32_t APB1RSTR;			// APB1 peripheral reset
  __vo uint32_t APB2RSTR; 			// APB2 peripheral reset
  uint32_t      RESERVED1[2];  		// reserved
  __vo uint32_t AHB1ENR;			// AHB1 peripheral clock enable reset
  __vo uint32_t AHB2ENR;			// AHB2 peripheral clock enable reset
  __vo uint32_t AHB3ENR;			// AHB3 peripheral clock enable reset
  uint32_t      RESERVED2;			// reserved
  __vo uint32_t APB1ENR;			// APB1 peripheral clock enable reset
  __vo uint32_t APB2ENR;			// APB2 peripheral clock enable reset
  uint32_t      RESERVED3[2]; 		// reserved
  __vo uint32_t AHB1LPENR;			// AHB1 peripheral clock enable low power mode reset
  __vo uint32_t AHB2LPENR;			// AHB2 peripheral clock enable low power mode reset
  __vo uint32_t AHB3LPENR;			// AHB3 peripheral clock enable low power mode reset
  uint32_t      RESERVED4;    		// reserved
  __vo uint32_t APB1LPENR;			// APB1 peripheral clock enable low power mode reset
  __vo uint32_t APB2LPENR;			// APB2 peripheral clock enable low power mode reset
  uint32_t      RESERVED5[2]; 		// reserved
  __vo uint32_t BDCR;				// backup domain
  __vo uint32_t CSR;				// control control & status
  uint32_t      RESERVED6[2];  		// reserved
  __vo uint32_t SSCGR;				// spread spectrum clock generation
  __vo uint32_t PLLI2SCFGR;			// PLLI2S configuration
} RCC_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;   		// memory remap
	__vo uint32_t PMC;   			// peripheral map configuration
	__vo uint32_t EXTICR[4];   		// external interrupt configuration
	uint32_t      RESERVED1[2];   	// reserved
	__vo uint32_t CMPCR;   			// compensation cell control
} SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;   			// interrupt mask
	__vo uint32_t EMR;				// event mask
	__vo uint32_t RTSR;				// rising trigger selection
	__vo uint32_t FTSR;				// falling trigger selection
	__vo uint32_t SWIER;			// software interrupt event
	__vo uint32_t PR;				// pending
} EXTI_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;				// control 1
	__vo uint32_t CR2;				// control 2
	__vo uint32_t SR;				// status
	__vo uint32_t DR;				// data
	__vo uint32_t CRCPR;			// CRC polynomial
	__vo uint32_t RXCRCR;			// RX CRC
	__vo uint32_t TXCRCR;			// TX CRC
	__vo uint32_t I2SCFGR;			// I2S config
	__vo uint32_t I2SPR;			// I2S prescaler
} SPI_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;				// control 1
  __vo uint32_t CR2;				// control 2
  __vo uint32_t OAR1;				// own address 1
  __vo uint32_t OAR2;				// own address 2
  __vo uint32_t DR;					// data
  __vo uint32_t SR1;				// status 1
  __vo uint32_t SR2;  				// status 2
  __vo uint32_t CCR;				// clock control
  __vo uint32_t TRISE;				// trise
  __vo uint32_t FLTR;				// fltr
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;				// status
	__vo uint32_t DR;				// data
	__vo uint32_t BRR;				// baud rate
	__vo uint32_t CR1;				// control 1
	__vo uint32_t CR2;				// control 2
	__vo uint32_t CR3;				// control 3
	__vo uint32_t GTPR;				// guard time & prescaler
} USART_RegDef_t;

/******************************************************************************
 ****************************PERIPHERAL DEFINITION ****************************
 ******************************************************************************/

/*
 * peripheral definition for GPIOx
 */
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_ADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_ADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_ADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_ADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_ADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_ADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_ADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_ADDR)
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_ADDR)

/*
 * peripheral definition for RCC
 */
#define RCC 		((RCC_RegDef_t*)RCC_ADDR)

/*
 * peripheral definition for SYSCFG
 */
#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_ADDR)

/*
 * peripheral definition for EXTI
 */
#define EXTI		((EXTI_RegDef_t*)EXTI_ADDR)

/*
 * peripheral definition for SPIx
 */
#define SPI1  		((SPI_RegDef_t*)SPI1_ADDR)
#define SPI2  		((SPI_RegDef_t*)SPI2_ADDR)
#define SPI3  		((SPI_RegDef_t*)SPI3_ADDR)

/*
 * peripheral definition for I2Cx
 */
#define I2C1  		((I2C_RegDef_t*)I2C1_ADDR)
#define I2C2  		((I2C_RegDef_t*)I2C2_ADDR)
#define I2C3  		((I2C_RegDef_t*)I2C3_ADDR)

/*
 * peripheral definition for USARTx
 */
#define USART1  	((USART_RegDef_t*)USART1_ADDR)
#define USART2  	((USART_RegDef_t*)USART2_ADDR)
#define USART3  	((USART_RegDef_t*)USART3_ADDR)
#define UART4  		((USART_RegDef_t*)UART4_ADDR)
#define UART5  		((USART_RegDef_t*)UART5_ADDR)
#define USART6  	((USART_RegDef_t*)USART6_ADDR)

/******************************************************************************
 *****************************CLOCK ENABLE MACROS *****************************
 ******************************************************************************/

/*
 * clock enable macros for GPIOx
 */
#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/*
 * clock enable macros for SYSCFG
 */
#define SYSCFG_PCLK_EN() 	(RCC->APB2ENR |= (1 << 14))

/*
 * clock enable macros for SPIx
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/*
 * clock enable macros for I2Cx
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * clock enable macros for USARTx
 */
#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  	(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB1ENR |= (1 << 5))

/******************************************************************************
 *****************************CLOCK DISABLE MACROS*****************************
 ******************************************************************************/

/*
 * clock disable macros for GPIOx
 */
#define GPIOA_PCLK_DIS()    (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DIS()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DIS()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DIS()	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DIS()	(RCC->AHB1ENR &= ~(1 << 8))

/*
 * clock disable macros for SYSCFG
 */
#define SYSCFG_PCLK_DIS() 	(RCC->APB2ENR &= ~(1 << 14))

/*
 * clock disable macros for SPIx
 */
#define SPI1_PCLK_DIS()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 15))

/*
 * clock disable macros for I2Cx
 */
#define I2C1_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DIS()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * clock disable macros for USARTx
 */
#define USART1_PCLK_DIS() 	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DIS() 	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DIS()  	(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DIS()	(RCC->APB2ENR &= ~(1 << 5))

/******************************************************************************
 ***************************RESET PERIPHERAL MACROS ***************************
 ******************************************************************************/

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 12)); (RCC->AHB1RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 14)); (RCC->AHB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 15)); (RCC->AHB1RSTR &= ~(1 << 15));}while(0)

/*
 *  Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 21)); (RCC->AHB1RSTR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 22)); (RCC->AHB1RSTR &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 23)); (RCC->AHB1RSTR &= ~(1 << 23));}while(0)

/*
 *  Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET()	do{(RCC->APB2RSTR |= (1 <<  4)); (RCC->AHB1RSTR &= ~(1 <<  4));}while(0)
#define USART2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 17)); (RCC->AHB1RSTR &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 18)); (RCC->AHB1RSTR &= ~(1 << 18));}while(0)
#define UART4_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 19)); (RCC->AHB1RSTR &= ~(1 << 19));}while(0)
#define UART5_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 20)); (RCC->AHB1RSTR &= ~(1 << 20));}while(0)
#define USART6_REG_RESET()	do{(RCC->APB1RSTR |= (1 <<  5)); (RCC->AHB1RSTR &= ~(1 <<  5));}while(0)

/******************************************************************************
 **********************************INTERRUPT **********************************
 ******************************************************************************/

/*
 * IRQ(Interrupt Request) numbers
 */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4			84
#define IRQ_NO_I2C1_EV     	31
#define IRQ_NO_I2C1_ER     	32
#define IRQ_NO_I2C2_EV     	33
#define IRQ_NO_I2C2_ER     	34
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71
#define IRQ_NO_I2C3_EV     	79
#define IRQ_NO_I2C3_ER     	80

/*
 * all the possible priority levels
 */
#define NVIC_IRQ_PRI0    	0
#define NVIC_IRQ_PRI1   	1
#define NVIC_IRQ_PRI2  	 	2
#define NVIC_IRQ_PRI3    	3
#define NVIC_IRQ_PRI4    	4
#define NVIC_IRQ_PRI5    	5
#define NVIC_IRQ_PRI6    	6
#define NVIC_IRQ_PRI7    	7
#define NVIC_IRQ_PRI8    	8
#define NVIC_IRQ_PRI9    	9
#define NVIC_IRQ_PRI10   	10
#define NVIC_IRQ_PRI11   	11
#define NVIC_IRQ_PRI12   	12
#define NVIC_IRQ_PRI13    	13
#define NVIC_IRQ_PRI14   	14
#define NVIC_IRQ_PRI15    	15

/******************************************************************************
 *******************************BIT POS DEF SPI *******************************
 ******************************************************************************/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA					0
#define SPI_CR1_CPOL					1
#define SPI_CR1_MSTR					2
#define SPI_CR1_BR						3
#define SPI_CR1_SPE						6
#define SPI_CR1_LSBFIRST				7
#define SPI_CR1_SSI						8
#define SPI_CR1_SSM 					9
#define SPI_CR1_RXONLY					10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************
 *******************************BIT POS DEF I2C *******************************
 ******************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				0
#define I2C_OAR1_ADD71 				 	1
#define I2C_OAR1_ADD98  			 	8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************
 ******************************BIT POS DEF USART ******************************
 ******************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */
#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#include <stm32f407xx_gpio_driver.h>
#include <stm32f407xx_spi_driver.h>
#include <stm32f407xx_i2c_driver.h>
#include <stm32f407xx_usart_driver.h>
#endif /* INC_STM32F407XX_H_ */
