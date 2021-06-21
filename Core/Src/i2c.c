/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{
  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**I2C1 GPIO Configuration
  PA13   ------> I2C1_SCL
  PA14   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /** I2C Initialization
  */
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x20A0C4DF;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);

}

/* USER CODE BEGIN 1 */
//I2C发送定长数据,最大只能发送256字节
/**
  * @brief  Handles I2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @rmtoll CR2          SADD          LL_I2C_HandleTransfer\n
  *         CR2          ADD10         LL_I2C_HandleTransfer\n
  *         CR2          RD_WRN        LL_I2C_HandleTransfer\n
  *         CR2          START         LL_I2C_HandleTransfer\n
  *         CR2          STOP          LL_I2C_HandleTransfer\n
  *         CR2          RELOAD        LL_I2C_HandleTransfer\n
  *         CR2          NBYTES        LL_I2C_HandleTransfer\n
  *         CR2          AUTOEND       LL_I2C_HandleTransfer\n
  *         CR2          HEAD10R       LL_I2C_HandleTransfer
  * @param  I2Cx I2C Instance.
  * @param  SlaveAddr Specifies the slave address to be programmed.
  * @param  SlaveAddrSize This parameter can be one of the following values:
  *         @arg @ref LL_I2C_ADDRSLAVE_7BIT
  *         @arg @ref LL_I2C_ADDRSLAVE_10BIT
  * @retval Data length
  */
uint16_t	API_I2CxMasterSend(I2C_TypeDef *I2Cx, uint32_t SlaveAddr, uint32_t SlaveAddrSize,uint16_t RegAddr, uint16_t RegAddrSize, uint16_t DataLen,const uint8_t *pData)
{
    volatile    uint16_t    I2C_timeout = 2000;
	uint16_t	iLoop=0;
	
    //I2C1->CR2 = I2C_CR2_AUTOEND | (DataLen<<16) | (SLAVE_ADDRESS<<1);
    I2Cx->CR2 = I2C_CR2_AUTOEND | ((DataLen+1)<<16) | (SlaveAddr) | (SlaveAddrSize);
	I2Cx->CR2 &= ~(I2C_CR2_RD_WRN);
	I2Cx->CR2 |= I2C_CR2_START;
    // RBYTE OFFSET ADDRESS
    I2C_timeout = 2000;
    while( ((I2Cx->ISR & I2C_ISR_TXE) !=  I2C_ISR_TXE) && (--I2C_timeout) );
    if( I2C_timeout == 0 ){
        I2C_timeout = 20;
        I2Cx->CR1 &= ~(I2C_CR1_PE);
        while( --I2C_timeout );
        I2Cx->CR1 |= I2C_CR1_PE;
        return 0;
    }
	if (RegAddrSize == LL_I2C_REG_8BIT)
	{
		I2Cx->TXDR  = (RegAddr & 0xFF);
	}
	else
	{
		I2Cx->TXDR = REG_ADDR_MSB(RegAddr);
    
		I2C_timeout = 2000;
		while( ((I2Cx->ISR & I2C_ISR_TXE) !=  I2C_ISR_TXE) && (--I2C_timeout) );
		if( I2C_timeout == 0 ){
			I2C_timeout = 20;
			I2Cx->CR1 &= ~(I2C_CR1_PE);
			while( --I2C_timeout );
			I2Cx->CR1 |= I2C_CR1_PE;
			return 0;
		}
		I2Cx->TXDR = REG_ADDR_LSB(RegAddr);
    }
    
    // DATA WORD n
	for( iLoop=0; iLoop<DataLen; iLoop++ )
    {
        I2C_timeout = 2000;
        while( ((I2Cx->ISR & I2C_ISR_TXE) !=  I2C_ISR_TXE) && (--I2C_timeout) );
        if( I2C_timeout == 0 ){
			I2C_timeout = 20;
			I2Cx->CR1 &= ~(I2C_CR1_PE);
			while( --I2C_timeout );
			I2Cx->CR1 |= I2C_CR1_PE;
			return 0;
        }
		I2Cx->TXDR = pData[iLoop];
	}
    
    return DataLen;
}

//I2C接收定长数据,I2C发送定长数据,最大只能接收256字节
uint16_t	API_I2CxMasterRecv(I2C_TypeDef *I2Cx,uint32_t SlaveAddr, uint32_t SlaveAddrSize,uint16_t RegAddr, uint16_t RegAddrSize, uint16_t DataLen, __IO uint8_t *pData)
{
    volatile    uint16_t    I2C_timeout = 2000;
	uint16_t	iLoop=0;
    
    I2Cx->CR2 =  I2C_CR2_AUTOEND | (1<<16) | (SlaveAddr) | (SlaveAddrSize);// I2C_CR2_RELOAD
	I2Cx->CR2 &= ~(I2C_CR2_RD_WRN);
//    I2C3->CR2 &= ~(I2C_CR2_RELOAD);
	I2Cx->CR2 |= I2C_CR2_START;
    // RBYTE OFFSET ADDRESS
    I2C_timeout = 2000;
    while( ((I2Cx->ISR & I2C_ISR_TXE) !=  I2C_ISR_TXE) && (--I2C_timeout) );
    if( I2C_timeout == 0 ){
        I2C_timeout = 20;
        I2Cx->CR1 &= ~(I2C_CR1_PE);
        while( --I2C_timeout );
        I2Cx->CR1 |= I2C_CR1_PE;
        return 0;
    }
    if (RegAddrSize == LL_I2C_REG_8BIT)
	{
        I2Cx->TXDR = REG_ADDR_MSB(RegAddr); 
    }
    else
    {
        I2C_timeout = 2000;
        while( ((I2Cx->ISR & I2C_ISR_TXE) !=  I2C_ISR_TXE) && (--I2C_timeout) );
        if( I2C_timeout == 0 ){
            I2C_timeout = 20;
            I2Cx->CR1 &= ~(I2C_CR1_PE);
            while( --I2C_timeout );
            I2Cx->CR1 |= I2C_CR1_PE;
            return 0;
        }
        I2Cx->TXDR = REG_ADDR_LSB(RegAddr); 
    }

    I2C_timeout = 3000;//2112
    while( ((I2Cx->ISR & I2C_ISR_BUSY) ==  I2C_ISR_BUSY) && (--I2C_timeout) );
    if( I2C_timeout == 0 ){
        I2C_timeout = 20;
        I2Cx->CR1 &= ~(I2C_CR1_PE);
        while( --I2C_timeout );
        I2Cx->CR1 |= I2C_CR1_PE;
        return 0;
    }    
    
    //I2C1->CR2 = I2C_CR2_AUTOEND | (DataLen<<16) | (SLAVE_ADDRESS<<1);
	I2Cx->CR2 =  I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | (DataLen<<16) | (SlaveAddr) | (SlaveAddrSize);
	I2Cx->CR2 |= I2C_CR2_START;
	
	for( iLoop=0; iLoop<DataLen; iLoop++ )
    {
        I2C_timeout = 2000;
        while( ((I2Cx->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) && (--I2C_timeout) );
        if( I2C_timeout == 0 ){
			I2C_timeout = 20;
			I2Cx->CR1 &= ~(I2C_CR1_PE);
			while( --I2C_timeout );
			I2Cx->CR1 |= I2C_CR1_PE;
			return 0;
        }
		pData[iLoop] = I2Cx->RXDR;
	}
    
    return DataLen;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
