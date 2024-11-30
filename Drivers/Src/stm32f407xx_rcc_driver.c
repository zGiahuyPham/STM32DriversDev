/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Sep 5, 2024
 *      Author: ghp
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4 , 8, 16};

// This driver hasn't covered for PLL Output CLock yet.

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, tmp, ahbp, apb1p;

	clksrc = (RCC->CFGR >> 2) & 0x3;
	if (clksrc == 0) SystemClk = 16000000;
	else SystemClk = 8000000;

	tmp = (RCC->CFGR >> 4) & 0xF;
	if (tmp < 8) ahbp = 1;
	else ahbp = AHB_PreScaler[tmp - 8];

	tmp = ((RCC->CFGR >> 10) & 0x7);
	if (tmp < 4) apb1p = 1;
	else apb1p = APB1_PreScaler[tmp - 4];

	pclk1 = (SystemClk / ahbp) /apb1p;
	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, SystemClk;
	uint8_t clksrc, tmp, ahbp, apb2p;

	clksrc = (RCC->CFGR >> 2) & 0x3;
	if (clksrc == 0) SystemClk = 16000000;
	else SystemClk = 8000000;

	tmp = (RCC->CFGR >> 4) & 0xF;
	if (tmp < 8) ahbp = 1;
	else ahbp = AHB_PreScaler[tmp - 8];

	tmp = (RCC->CFGR >> 13) & 0x7;
	if (tmp < 4) apb2p = 1;
	else apb2p = APB1_PreScaler[tmp - 4];

	pclk2 = (SystemClk / ahbp) / apb2p;
	return pclk2;
}
