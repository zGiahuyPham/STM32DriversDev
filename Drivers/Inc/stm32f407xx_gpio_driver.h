/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Sep 5, 2024
 *      Author: ghp
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include <stm32f407xx.h>

/*
 * configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

} GPIO_Handle_t;

/*
 * @GPIO_PinNumber
 */
#define GPIO_PIN_NO_0  		0
#define GPIO_PIN_NO_1  		1
#define GPIO_PIN_NO_2  		2
#define GPIO_PIN_NO_3  		3
#define GPIO_PIN_NO_4  		4
#define GPIO_PIN_NO_5  		5
#define GPIO_PIN_NO_6  		6
#define GPIO_PIN_NO_7  		7
#define GPIO_PIN_NO_8  		8
#define GPIO_PIN_NO_9  		9
#define GPIO_PIN_NO_10  	10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12  	12
#define GPIO_PIN_NO_13 		13
#define GPIO_PIN_NO_14 		14
#define GPIO_PIN_NO_15 		15

/*
 * @GPIO_PinMode
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6

/*
 * @GPIO_PinSpeed
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPOI_SPEED_HIGH		3

/*
 * @GPIO_PinPuPdControl
 */
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * @GPIO_PinOPType
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1

/******************************************************************************
 ********************************API SUPPORTED ********************************
 ******************************************************************************/

/*
 * peripheral clock setup
 */
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t State);

/*
 * setup and reset
 */
void GPIO_Setup(GPIO_Handle_t *pGPIOHandle);
void GPIO_Reset(GPIO_RegDef_t *pGPIOx);

/*
 * data read and write
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
