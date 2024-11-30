/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Sep 5, 2024
 *      Author: ghp
 */

#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

void I2C_PCLKControl(I2C_RegDef_t *pI2Cx, uint8_t State)
{
	if (State == ENABLE)
	{
		if (pI2Cx == I2C1) I2C1_PCLK_EN(); else
		if (pI2Cx == I2C2) I2C2_PCLK_EN(); else
		if (pI2Cx == I2C3) I2C3_PCLK_EN();
	}
	else
	{
		if (pI2Cx == I2C1) I2C1_PCLK_DIS(); else
		if (pI2Cx == I2C2) I2C2_PCLK_DIS(); else
		if (pI2Cx == I2C3) I2C3_PCLK_DIS();
	}
}

void I2C_Setup(I2C_Handle_t *pI2CHandle)
{
	//enable the clock for the i2cx peripheral
	I2C_PCLKControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t tmpreg = 0;

	//ack control bit
	tmpreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tmpreg;

	//configure the FREQ field of CR2
	tmpreg = 0;
	tmpreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 = tmpreg & 0x3F;

   //program the device own address
	tmpreg = 0;
	tmpreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tmpreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tmpreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tmpreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
	} else
	{
		//mode is fast mode
		tmpreg |= (1 << 15);
		tmpreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
	}
	tmpreg |= (ccr_value & 0xFFF);
	pI2CHandle->pI2Cx->CCR = tmpreg;

	//TRISE Configuration
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		tmpreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;
	} else
	{
		//mode is fast mode
		tmpreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tmpreg & 0x3F);
}

void I2C_Reset(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1) I2C1_REG_RESET(); else
	if (pI2Cx == I2C2) I2C2_REG_RESET(); else
	if (pI2Cx == I2C3) I2C3_REG_RESET();
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t State)
{
	if (State == ENABLE) pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	else pI2Cx->CR1 &= ~(1 << 0);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName) return SET;
	return RESET;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t State)
{
	if (State == I2C_ACK_ENABLE) pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	else pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;

	//check for device mode
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear the ADDR flag (read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			//clear the ADDR flag (read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag (read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

void I2C_MasterTx(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until len becomes 0
	while(Len)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if (Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterRx(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave
	if (Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//generate STOP condition
		if (Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

    //procedure to read data from slave when Len > 1
	if (Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for(uint32_t i = Len; i; i--)
		{
			//wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if (Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}
			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}

	//re-enable ACKing
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t State)
{
	 if (State == ENABLE)
	 {
		 pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN) | (1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITERREN);
	 } else
	 {
		 pI2Cx->CR2 &= ~((1 << I2C_CR2_ITEVTEN) | (1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITERREN));
	 }
}

uint8_t I2C_MasterTxIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t state = pI2CHandle->TxRxState;

	if (state != I2C_BUSY_IN_TX && state != I2C_BUSY_IN_RX)
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN, ITEVFEN, ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITEVTEN) | (1 << I2C_CR2_ITERREN);
	}
	return state;
}

uint8_t I2C_MasterRxIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t state = pI2CHandle->TxRxState;

	if (state != I2C_BUSY_IN_TX && state != I2C_BUSY_IN_RX)
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN, ITEVFEN, ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITEVTEN) | (1 << I2C_CR2_ITERREN);
	}
	return state;
}

void I2C_CloseTx(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_CloseRx(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
}

void I2C_SlaveTx(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}

uint8_t I2C_SlaveRx(I2C_RegDef_t *pI2C)
{
    return (uint8_t)pI2C->DR;
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State)
{
	if (State == ENABLE)
	{
		if (IRQNumber <= 31) *NVIC_ISER0 |= (1 << IRQNumber); else
		if (IRQNumber < 64) *NVIC_ISER1 |= (1 << (IRQNumber % 32)); else
		if (IRQNumber < 96) *NVIC_ISER2 |= (1 << (IRQNumber % 64));
	} else
	{
		if (IRQNumber <= 31) *NVIC_ICER0 &= ~(1 << IRQNumber); else
		if (IRQNumber < 64) *NVIC_ICER1 &= ~(1 << (IRQNumber % 32)); else
		if (IRQNumber < 96) *NVIC_ICER2 &= ~(1 << (IRQNumber % 64));
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t tmp1, tmp2, tmp3;
	tmp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	tmp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if (tmp1 && tmp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
	}

	tmp3  = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if (tmp1 && tmp3)
	{
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	tmp3  = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if (tmp1 && tmp3)
	{
		// in Tx case we close the transmission
		// make sure that TXE is also set and Tx complete as Len = 0.
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX &&
			pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE) && !pI2CHandle->TxLen)
		{
			//1. generate the STOP condition
			if (pI2CHandle->Sr == I2C_DISABLE_SR)
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			//2. reset all the member elements of the handle structure.
			I2C_CloseTx(pI2CHandle);

			//3. notify the application about transmission complete
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
		}
	}

	tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if (tmp1 && tmp3)
	{
		//STOF flag is set
		//Clear the STOPF by reading SR1 (already done above) then write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if (tmp1 && tmp2 && tmp3)
	{
		//TXE flag is set
		//Check for device mode
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//master
			//We have to do the data transmission
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX && pI2CHandle->TxLen)
			{
				//1. load the data in to DR
				pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
				//2. decrement the TxLen
				pI2CHandle->TxLen--;
				//3. Increment the buffer address
				pI2CHandle->pTxBuffer++;
			}
		} else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		    }
		}
	}

	tmp3  = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if (tmp1 && tmp2 && tmp3)
	{
		//check device mode .
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//The device is master
			//RXNE flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if (pI2CHandle->RxSize == 1)
				{
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;
				}
				if (pI2CHandle->RxSize > 1)
				{
					if (pI2CHandle->RxLen == 2)
					{
						//clear the ack bit
						I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
					}
					//read DR
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;
				}

				if (!pI2CHandle->RxLen)
				{
					//close the I2C data reception and notify the application
					//1. generate the stop condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2 . Close the I2C rx
					I2C_CloseRx(pI2CHandle);

					//3. Notify the application
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}
			}
		} else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t tmp1, tmp2;
    //Know the status of  ITERREN control bit in the CR2
	tmp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);

	//Check for Bus error
	tmp1 = pI2CHandle->pI2Cx->SR1 & (1<< I2C_SR1_BERR);
	if (tmp1  && tmp2)
	{
		//This is Bus error
		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_BERR);
	}

	//Check for arbitration lost error
	tmp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO);
	if (tmp1 && tmp2)
	{
		//This is arbitration lost error
		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_ARLO);
	}

	//Check for ACK failure error

	tmp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if (tmp1 && tmp2)
	{
		//This is ACK failure error
	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_AF);
	}

	//Check for Overrun/underrun error
	tmp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if (tmp1 && tmp2)
	{
		//This is Overrun/underrun
	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_OVR);
	}

	//Check for Time out error
	tmp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if (tmp1 && tmp2)
	{
		//This is Time out error
	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_TIMEOUT);
	}
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	//This is a weak implementation . the user application may override this function.
}
