/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
/////////////////// GSM USART /////////////////////
#define GSM_RX_BUFFER_SIZE 64

#if (GSM_RX_BUFFER_SIZE > 256)
	typedef uint16_t gsm_rx_buffer_index_t;
#else
	typedef uint8_t gsm_rx_buffer_index_t;
#endif

int16_t gsm_available(void);
int16_t gsm_read(void);


#define DBG_RX_BUFFER_SIZE 64

#if (DBG_RX_BUFFER_SIZE > 256)
	typedef uint16_t dbg_rx_buffer_index_t;
#else
	typedef uint8_t dbg_rx_buffer_index_t;
#endif

	int16_t dbg_available(void);
	int16_t dbg_read(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
