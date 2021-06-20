/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define LL_I2C_REG_8BIT                0x00000000U
#define LL_I2C_REG_16BIT               0x00000001U          
/* For 16bit register address */
#define REG_ADDR_MSB(addr)             ((uint8_t)(addr>>8))
#define REG_ADDR_LSB(addr)             ((uint8_t)(addr&0xFF))
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
uint16_t	API_I2CxMasterSend(I2C_TypeDef *I2Cx,uint32_t SlaveAddr, uint32_t SlaveAddrSize,uint16_t RegAddr, uint16_t RegAddrSize, uint16_t DataLen, const uint8_t *pData);
uint16_t	API_I2CxMasterRecv(I2C_TypeDef *I2Cx,uint32_t SlaveAddr, uint32_t SlaveAddrSize,uint16_t RegAddr, uint16_t RegAddrSize, uint16_t DataLen, __IO uint8_t *pData);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
