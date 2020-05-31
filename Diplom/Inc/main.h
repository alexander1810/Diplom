/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define BUF_SIZE 256


#define	GAMMA  0.5
#define	RESISTENCE  9000
#define	RES_ON1LUX  75000

#define POOLTEMP_UPLIM 26
#define POOLTEMP_LOWLIM 24

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

enum STATUS {
	STOP,
	SETTINGS,
	START
};
typedef enum{
	FALSE,
	TRUE
}FLAG;



typedef struct{
	 uint8_t FillingFactor_PumpPWM;
	 uint8_t FillingFactor_LedPWM;
}Filling_Factor_PWM_t;

typedef struct{
	float Foto_Resistence;
	uint16_t Lux;
}ResultValue_Foto_t;

typedef struct{
	uint8_t Rh_dht11_byte1;
	uint8_t Rh_dht11_byte2;
	uint8_t Temp_dht11_byte1;
	uint8_t Temp_dht11_byte2;

	uint8_t Temp_ds18b20_byte1;
	uint8_t Temp_ds18b20_byte2;
}bytes_Sensors_t;


 typedef struct{
	 uint16_t SUM_DHT11;
	 uint16_t RH_DHT11;
	 uint16_t TEMP_DHT11;

	 uint16_t TEMP_DS18B20;
 }result_valuesOfSensors_t;

 typedef struct{
	 uint8_t GHTemp_UpperLimit;
	 uint8_t GHRh_UpperLimit;
	 uint8_t Ph_UpperLimit;
 }Sensors_UpperLimitParameters_t;

 typedef struct{
	 uint8_t GHTemp_LowerLimit;
	 uint8_t GHRh_LowerLimit;
	 uint8_t Ph_LowerLimit;
 }Sensors_LowerLimitParameters_t;

 typedef struct{
	 uint16_t Foto_CriticalValue;
	 uint8_t ContainerRh_CriticalValue;
	 uint16_t CO_CriticalValue;
 }Sensors_CriticalParameters_t;

 typedef struct{
	 uint8_t TimePipe_PumpDelay;
	 uint8_t TimePool_PumpDelay;
	 uint8_t TimeDay_Delay;
 }Time_Delays_t;

 typedef struct{
	 uint8_t Second_Counter;
	 uint8_t PoolPump_Minute_Counter;
	 uint8_t PipePump_Minute_Counter;
	 uint8_t DayDelay_Minute_Counter;
	 uint8_t DayDelay_Hour_Counter;
 }Installation_TimersCountrs_t;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_SWITCH_Pin GPIO_PIN_7
#define PWM_SWITCH_GPIO_Port GPIOA
#define CONTROL_Cooler_Pin GPIO_PIN_0
#define CONTROL_Cooler_GPIO_Port GPIOB
#define CONTROL_Hum_Pin GPIO_PIN_1
#define CONTROL_Hum_GPIO_Port GPIOB
#define CONTROL_WPoolPump_Pin GPIO_PIN_10
#define CONTROL_WPoolPump_GPIO_Port GPIOB
#define CONTROL_WH_Pin GPIO_PIN_11
#define CONTROL_WH_GPIO_Port GPIOB
#define CONTROL_CH_HEATER_Pin GPIO_PIN_8
#define CONTROL_CH_HEATER_GPIO_Port GPIOA
#define CONTROL_FAN_Pin GPIO_PIN_9
#define CONTROL_FAN_GPIO_Port GPIOA
#define PWM_PUMP_Pin GPIO_PIN_10
#define PWM_PUMP_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
