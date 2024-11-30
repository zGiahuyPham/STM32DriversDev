/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Sep 5, 2024
 *      Author: ghp
 */

#include "stm32f407xx_spi_driver.h"

void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t State)
{
	if (State == ENABLE)
	{
		if (pSPIx == SPI1) SPI1_PCLK_EN(); else
		if (pSPIx == SPI2) SPI2_PCLK_EN(); else
		if (pSPIx == SPI3) SPI3_PCLK_EN();
	}
	else
	{
		if (pSPIx == SPI1) SPI1_PCLK_DIS(); else
		if (pSPIx == SPI2) SPI2_PCLK_DIS(); else
		if (pSPIx == SPI3) SPI3_PCLK_DIS();
	}
}

void SPI_Setup(SPI_Handle_t *pSPIHandle)
{
	SPI_PCLKControl(pSPIHandle->pSPIx, ENABLE); //peripheral clock enable
	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);	//bidi mode should be cleared
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		tempreg |= (1 << SPI_CR1_BIDIMODE); 	//bidi mode should be set
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);	//bidi mode should be cleared
		tempreg |= (1 << SPI_CR1_RXONLY); 		//rxonly bit must be set
	}

	//3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_Reset(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1) SPI1_REG_RESET(); else
	if (pSPIx == SPI2) SPI2_REG_RESET(); else
	if (pSPIx == SPI3) SPI3_REG_RESET();
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if (pSPIx->SR & FlagName) return SET;
	return RESET;
}

void SPI_Tx(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == RESET);

		//2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) //16 bit DFF
		{
			//1. load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2;
			(uint16_t*)pTxBuffer++;
		} else //8 bit DFF
		{
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_Rx(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == RESET);

		//2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) //16 bit DFF
		{
			//1. load the data from DR to Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len -= 2;
			(uint16_t*)pRxBuffer++;
		} else //8 bit DFF
		{
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_TxIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_RxIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t State)
{
	if (State == ENABLE) pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	else pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t State)
{
	if (State == ENABLE) pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	else pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}

void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t State)
{
	if (State == ENABLE) pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	else pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t tmp1, tmp2;
	//first lets check for TXE
	tmp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	tmp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (tmp1 && tmp2) //handle TXE
	{
		// check the DFF bit in CR1
		if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) //16 bit DFF
		{
			//1. load the data in to the DR
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen -= 2;
			(uint16_t*)pSPIHandle->pTxBuffer++;
		} else //8 bit DFF
		{
			pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}
		if (!pSPIHandle->TxLen)
		{
			//TxLen is zero , so close the spi transmission and inform the application that
			//TX is over.
			//this prevents interrupts from setting up of TXE flag
			pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
			pSPIHandle->pTxBuffer = NULL;
			pSPIHandle->TxLen = 0;
			pSPIHandle->TxState = SPI_READY;

			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
		}
	}

	// check for RXNE
	tmp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	tmp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (tmp1 && tmp2) //handle RXNE
	{
		// check the DFF bit in CR1
		if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) //16 bit DFF
		{
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRxBuffer++;
			pSPIHandle->pRxBuffer++;
		} else //8 bit
		{
			*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}
		if (!pSPIHandle->RxLen)
		{
			//reception is complete
			pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
			pSPIHandle->pRxBuffer = NULL;
			pSPIHandle->RxLen = 0;
			pSPIHandle->RxState = SPI_READY;

			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}
	}

	// check for ovr flag
	tmp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	tmp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (tmp1 && tmp2)
	{
		//handle ovr error
		uint8_t tmp;
		//1. clear the ovr flag
		if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
		{
			tmp = pSPIHandle->pSPIx->DR;
			tmp = pSPIHandle->pSPIx->SR;
		}
		(void)tmp;
		//2. inform the application
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
	}
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//This is a weak implementation . the user application may override this function.
}
