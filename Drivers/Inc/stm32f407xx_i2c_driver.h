/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Sep 5, 2024
 *      Author: ghp
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

} I2C_Config_t;

/*
 *Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer;
	uint8_t 		*pRxBuffer;
	uint32_t 		TxLen;
	uint32_t 		RxLen;
	uint8_t 		TxRxState;
	uint8_t 		DevAddr;
    uint32_t        RxSize;
    uint8_t         Sr;
} I2C_Handle_t;

/*
 * I2C application states
 */
#define I2C_READY 				0
#define I2C_BUSY_IN_RX 			1
#define I2C_BUSY_IN_TX 			2

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 		100000
#define I2C_SCL_SPEED_FM4K 		400000
#define I2C_SCL_SPEED_FM2K 	 	200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9     	1

/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE   			(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   		(1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB				(1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  			(1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   			(1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 			(1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 			(1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 			(1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 			(1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  			(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 			(1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 		(1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  		RESET
#define I2C_ENABLE_SR   		SET

/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ER_BERR 	 		3
#define I2C_ER_ARLO  			4
#define I2C_ER_AF    			5
#define I2C_ER_OVR   			6
#define I2C_ER_TIMEOUT 			7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

/******************************************************************************
 ********************************API SUPPORTED ********************************
 ******************************************************************************/

/*
 * peripheral clock setup
 */
void I2C_PCLKControl(I2C_RegDef_t *pI2Cx, uint8_t State);

/*
 * setup and reset
 */
void I2C_Setup(I2C_Handle_t *pI2CHandle);
void I2C_Reset(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */
void I2C_MasterTx(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterRx(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterTxIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterRxIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);


void I2C_SlaveTx(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveRx(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t State);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t State);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t State);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
