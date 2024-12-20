/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Sep 5, 2024
 *      Author: ghp
 */

#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock and Mantissa and Fraction values
	uint32_t PCLKx, M_part,F_part, usartdiv, tmp = 0;

	//Get the value of APB bus clock in to the variable PCLKx
	if (pUSARTx == USART1 || pUSARTx == USART6) PCLKx = RCC_GetPCLK2Value();
	else PCLKx = RCC_GetPCLK1Value();

	//Check for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	} else
	{
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv / 100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tmp |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		F_part = (((F_part * 8)+ 50) / 100) & ((uint8_t)0x07);
	} else
	{
	   //over sampling by 16
	   F_part = (((F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tmp |= F_part;

	//copy the value of tmp in to BRR register
	pUSARTx->BRR = tmp;
}

void USART_PCLKControl(USART_RegDef_t *pUSARTx, uint8_t State)
{
	if (State == ENABLE)
	{
		if (pUSARTx == USART1) USART1_PCLK_EN(); else
		if (pUSARTx == USART2) USART2_PCLK_EN(); else
		if (pUSARTx == USART3) USART3_PCLK_EN(); else
		if (pUSARTx == UART4)  UART4_PCLK_EN();  else
		if (pUSARTx == UART5)  UART5_PCLK_EN();  else
		if (pUSARTx == USART6) USART6_PCLK_EN();
	}
	else
	{
		if (pUSARTx == USART1) USART1_PCLK_DIS(); else
		if (pUSARTx == USART2) USART2_PCLK_DIS(); else
		if (pUSARTx == USART3) USART3_PCLK_DIS(); else
		if (pUSARTx == UART4)  UART4_PCLK_DIS();  else
		if (pUSARTx == UART5)  UART5_PCLK_DIS();  else
		if (pUSARTx == USART6) USART6_PCLK_DIS();
	}
}

void USART_Setup(USART_Handle_t *pUSARTHandle)
{
	uint32_t tmp = 0;

	//Implement the code to enable the Clock for given USART peripheral
	 USART_PCLKControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tmp |= (1 << USART_CR1_RE);
	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tmp |= (1 << USART_CR1_TE);

	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tmp |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

    //Implement the code to configure the Word length configuration item
	tmp |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

    //Configuration of parity control bit fields
	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tmp |= (1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control
	} else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		//Implement the code to enable the parity control
	    tmp |= (1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tmp |= (1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tmp;

	tmp = 0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tmp |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tmp;

	tmp = 0;

	//Configuration of USART hardware flow control
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tmp |= (1 << USART_CR3_CTSE);
	} else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tmp |= (1 << USART_CR3_RTSE);

	} else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tmp |= (1 << USART_CR3_CTSE);
		tmp |= (1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tmp;

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);
}

void USART_Reset(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx == USART1) USART1_REG_RESET(); else
	if (pUSARTx == USART2) USART2_REG_RESET(); else
	if (pUSARTx == USART3) USART3_REG_RESET(); else
	if (pUSARTx == UART4)  UART4_REG_RESET();  else
	if (pUSARTx == UART5)  UART5_REG_RESET();  else
	if (pUSARTx == USART6) USART6_REG_RESET();
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t State)
{
	if (State == ENABLE) pUSARTx->CR1 |= (1 << 13);
	else pUSARTx->CR1 &= ~(1 << 13);
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName)
{
    if (pUSARTx->SR & FlagName) return SET;
    return RESET;
}

void USART_Tx(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			pUSARTHandle->pUSARTx->DR = (*((uint16_t*)pTxBuffer) & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
			}
			//Parity bit is used in this transfer . so 8bits of user data will be sent
			//The 9th bit will be replaced by parity bit by the hardware
			pTxBuffer++;
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

void USART_Rx(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame
			//Now, check are we using USART_ParityControl control or not
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 9bits will be of user data
				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer += 2;
			}
			else
			{
				//Parity is used, so 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame
			//Now, check are we using USART_ParityControl control or not
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data
				//read 8 bits from DR
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);

			}

			//Now , increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

uint8_t USART_TxIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pUSARTHandle->TxBusyState;

	if (state != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE and TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE) | (1 << USART_CR1_TCIE);
	}

	return state;
}

uint8_t USART_RxIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pUSARTHandle->RxBusyState;

	if (state != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return state;
}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State)
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t tmp1, tmp2, tmp3;

    //Implement the code to check the state of TC bit in the SR
	tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	tmp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if (tmp1 && tmp2)
	{
		//this interrupt is because of TC
		//close transmission and call application callback if TxLen is zero
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if (!pUSARTHandle->TxLen)
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	//Implement the code to check the state of TXE bit in the SR
	tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	tmp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);


	if (tmp1 && tmp2)
	{
		//this interrupt is because of TXE

		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if (pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pUSARTHandle->pUSARTx->DR = (*((uint16_t*)pUSARTHandle) & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;
				}

			}
			if (!pUSARTHandle->TxLen)
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	//Implement the code to check the state of RXNE bit in the SR
	tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);

	//Implement the code to check the state of RXNEIE bit in CR1
	tmp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);


	if (tmp1 && tmp2)
	{
		//this interrupt is because of rxne
		if (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if (pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen --;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
					}
					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}
			} //if of >0
			if (!pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	//Note : CTS feature is not applicable for UART4 and UART5
	//Implement the code to check the status of CTS bit in the SR
	tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR3
	tmp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5)
	tmp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if (tmp1 && tmp2 && tmp3)
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

	//Implement the code to check the status of IDLE flag bit in the SR
	tmp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	tmp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if (tmp1 && tmp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		tmp1 = pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	//Implement the code to check the status of ORE flag in the SR
	tmp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	tmp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if (tmp1 && tmp2)
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//We dont discuss multibuffer communication in this course. please refer to the RM
	//The blow code will get executed in only if multibuffer mode is used.

	tmp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

	if (tmp2)
	{
		tmp1 = pUSARTHandle->pUSARTx->SR;
		if (tmp1 & (1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}
		if (tmp1 & (1 << USART_SR_NE))
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
		}
		if (tmp1 & (1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}
}

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
{
	//This is a weak implementation . the user application may override this function.
}
