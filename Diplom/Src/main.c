/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ringbuffer_dma.h"
#include "dwt.h"
#include "dht_22.h"
#include "ds18b20.h"
#include "rcc.h"
//#include "stm32f1xx_it.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RingBuffer_DMA rx_buf;

uint8_t rx[BUF_SIZE]; // Array for DMA to save Rx bytes
uint32_t rx_count = 0;
uint32_t lastTick = 0;

enum STATUS status_Flag = STOP;
FLAG Write_LimitsFlag = FALSE;
FLAG Relay_PWMFlag = FALSE;
FLAG Pump_TimingFlag = FALSE;
FLAG Status_WorkingRelayFlag = FALSE;


char cmd[60];
uint8_t icmd = 0;
/* Array for Tx messages */
uint8_t tx[100];

char commandStr[30];
char Lower_Buffer[20];
char Upper_Buffer[20];
char Critical_Buffer[20];

uint16_t adc_buf[3];// ADC value
uint16_t sensors_Data[3];
uint8_t pHSensorsValue = 0;
uint32_t Resistance_MQ135;

extern uint8_t Night_Flag;


uint32_t array_ADC_CH[3] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6};

const char *GHTEMP = "GHTEMP=";
const char *GHRH = "GHRH=";
const char *PH = "PH=";
const char *PWMPUMP = "PWMPUMP=";
const char *LIGHT = "LIGHT=";
const char *CRIT_RH = "CRIT_RH=";
const char *CRIT_CO = "CRIT_CO=";
const char *PIPE_DELAY = "PIPE_DELAY=";
const char *POOL_DELAY = "POOL_DELAY=";

bytes_Sensors_t bytesOfDHT11;
bytes_Sensors_t bytesOfDS18B20;
result_valuesOfSensors_t valuesOfDHT11;
result_valuesOfSensors_t valuesOfDS18B20;

Sensors_UpperLimitParameters_t UpLimit = { 30, 90, 8 };
Sensors_LowerLimitParameters_t LowLimit = { 15, 20, 4};
Sensors_CriticalParameters_t CriticalParameters = { 1500, 80, 2500 };
Time_Delays_t Delays = {0, 0, 0};
ResultValue_Foto_t Foto_Result = {0,0};

Filling_Factor_PWM_t Filling_Factor = {0,0};
Installation_TimersCountrs_t Timer_Counter = {0,0,0,0,0};

int Value_Settings = 0;
uint8_t millisecondsDelay_PWM = 0;
extern uint8_t Delay_PWMFlag;

uint8_t Presence = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void Process_Command(char * command);
void Info_Sensors();
void Measure_DigitalSensors(void);
void Transforming_DataAnalogSensors(void);
void Set_LimitSettings(void);
void Answer_OK(void);
void Answer_UnknownParameter(void);
void write_Limits(void);
void Stop_Periphery(void);
void Start_Periphery(void);
void Relay_Algorithm(void);
void PWM_Algorithm(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  RingBuffer_DMA_Init(&rx_buf, huart2.hdmarx, rx, BUF_SIZE);
  HAL_UART_Receive_DMA(&huart2, rx, BUF_SIZE);

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

  DWT_DelayUpdate();
  DWT_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(Delay_PWMFlag){
		  DWT_Delay_ms((uint32_t)millisecondsDelay_PWM);
		  if(millisecondsDelay_PWM != 10)	HAL_GPIO_WritePin(GPIOA, PWM_PUMP_Pin,0);
		  Delay_PWMFlag = 0;
	  }
	  switch(status_Flag)
	  {
	  case SETTINGS:
		  if (HAL_GetTick() - lastTick > 1000)
		  {
			  //Measure_DigitalSensors();
			  Info_Sensors();
			  write_Limits();
			  lastTick = HAL_GetTick();
		  }
		  break;
	  case START:
		  Start_Periphery();
		  Relay_Algorithm();
		  PWM_Algorithm();
		  break;
	  case STOP:
		  Stop_Periphery();
		  break;
	  }

	  Transforming_DataAnalogSensors();

		rx_count = RingBuffer_DMA_Count(&rx_buf);
		while (rx_count--) {
			uint8_t b = RingBuffer_DMA_GetByte(&rx_buf);
			if (b == '\n') {
				cmd[icmd] = 0;
				icmd = 0;
				int i = 0;
				memset(commandStr,0,sizeof(commandStr));
				while(cmd[i]){
					commandStr[i]=cmd[i];
					i++;
				}
				Set_LimitSettings();
				Process_Command(cmd);
			} else if (b == '\r') {
				continue;
			} else {
				cmd[icmd++] = b;
			}
		}
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
void Process_Command(char * command)
{
	if(strcmp(command,"STOP") == 0)
	{
	  	status_Flag = STOP;
	  	sprintf((char *) tx,"Installation STOP\r\n");
	}
	else if(strcmp(command,"SETTINGS") == 0)
	{
	  	status_Flag = SETTINGS;
	  	sprintf((char *) tx,"Installation settings\r\n");
	}
	else if((strcmp(command,"WRON_LIMITS") == 0) && (status_Flag == SETTINGS))
	{
		Write_LimitsFlag = TRUE;
	}
	else if((strcmp(command,"WROFF_LIMITS") == 0) && (status_Flag == SETTINGS))
	{
		Write_LimitsFlag = FALSE;
	}
	else if(strcmp(command,"START") == 0)
	{
	  	status_Flag = START;
	  	sprintf((char *) tx, "Installation START\r\n");
		millisecondsDelay_PWM = Filling_Factor.FillingFactor_PumpPWM/10;
	}
  	else
  		sprintf((char *) tx, "command unknown\r\n");
  	HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
}

void Info_Sensors()
{
		sprintf((char *) tx, "GH_T=%dC, GH_RH=%d%%, S_T=%dC\r\n", valuesOfDHT11.TEMP_DHT11, valuesOfDHT11.RH_DHT11, valuesOfDS18B20.TEMP_DS18B20);
		HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);

		sprintf((char *) tx, "CO2=%d ppm, PH=%d pH, FOTO=%d Lx\r\n\r\n", sensors_Data[0], sensors_Data[1], sensors_Data[2]);
		HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
}

void Measure_DigitalSensors(void)
{
	DHT11_Start();
	Presence = DHT11_Check_Response();
	bytesOfDHT11.Rh_dht11_byte1 = DHT11_Read ();
	bytesOfDHT11.Rh_dht11_byte2 = DHT11_Read ();
	bytesOfDHT11.Temp_dht11_byte1 = DHT11_Read ();
	bytesOfDHT11.Temp_dht11_byte2 = DHT11_Read ();

	valuesOfDHT11.SUM_DHT11 = DHT11_Read();
	valuesOfDHT11.TEMP_DHT11 = bytesOfDHT11.Temp_dht11_byte1;
	valuesOfDHT11.RH_DHT11 = bytesOfDHT11.Rh_dht11_byte1;

	Presence = DS18B20_Start ();
	HAL_Delay (1);
	DS18B20_Write (0xCC);  // skip ROM
	DS18B20_Write (0x44);  // convert t
	HAL_Delay (800);

	Presence = DS18B20_Start ();
	HAL_Delay(1);
	DS18B20_Write (0xCC);  // skip ROM
	DS18B20_Write (0xBE);  // Read Scratch-pad

	bytesOfDS18B20.Temp_ds18b20_byte1 = DS18B20_Read();
	bytesOfDS18B20.Temp_ds18b20_byte2 = DS18B20_Read();

	valuesOfDS18B20.TEMP_DS18B20 = (bytesOfDS18B20.Temp_ds18b20_byte2<<8)|bytesOfDS18B20.Temp_ds18b20_byte1;
	valuesOfDS18B20.TEMP_DS18B20 = valuesOfDS18B20.TEMP_DS18B20/16;

}


void Transforming_DataAnalogSensors(void)
{

	for(int i = 0; i < 3; i++)
	{
		adc_buf[i] = ADC_GetValue(array_ADC_CH[i]);
	}
	sensors_Data[0] = adc_buf[0]/16;
	sensors_Data[1] = (14 * adc_buf[1])/4096;
	Foto_Result.Foto_Resistence = (adc_buf[2]*RESISTENCE)/(4095-adc_buf[2]);
	sensors_Data[2] = pow(10, (log10(RES_ON1LUX/Foto_Result.Foto_Resistence)/GAMMA));
}

void Answer_OK(void)
{
	sprintf((char *) tx, "OK\r\n");
	HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
}

void Answer_UnknownParameter(void)
{
	sprintf((char *) tx, "Uknown Parameter\r\n");
	HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
}

void write_Limits()
{
	if(Write_LimitsFlag == TRUE)
	{
		sprintf((char *) tx, "GHTEMP=[%d,%d]C;\r\n", LowLimit.GHTemp_LowerLimit, UpLimit.GHTemp_UpperLimit);
		HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
		sprintf((char *) tx, "GHRH=[%d,%d]%%;\r\n",   LowLimit.GHRh_LowerLimit, UpLimit.GHRh_UpperLimit);
		HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
		sprintf((char *) tx, "PH=[%d,%d];\r\n",     LowLimit.Ph_LowerLimit, UpLimit.Ph_UpperLimit);
		HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
		sprintf((char *) tx, "PWMPUMP=%d%% ;\r\n", 	Filling_Factor.FillingFactor_PumpPWM);
		HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
		sprintf((char *) tx, "LIGHT=%d Lx;\r\n", 	CriticalParameters.Foto_CriticalValue);
		HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
		sprintf((char *) tx, "CRIT_CO=%d ppm;\r\n", 	CriticalParameters.CO_CriticalValue);
		HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
		sprintf((char *) tx, "PIPE_DELAY=%d min;\r\n", 		Delays.TimePipe_PumpDelay);
		HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
		sprintf((char *) tx, "POOL_DELAY=%d min;\r\n\r\n", 		Delays.TimePool_PumpDelay);
		HAL_UART_Transmit(&huart2, tx, strlen((char *) tx), 100);
	}
}


void Set_LimitSettings()
{
	if(status_Flag == SETTINGS)
	{
		if (strncmp(commandStr, GHTEMP, 7)==0)
		{
			strncpy(Lower_Buffer,commandStr+7,2);
			strncpy(Upper_Buffer,commandStr+10,2);

			Value_Settings = atoi(Lower_Buffer);
			if((Value_Settings < 15) || (Value_Settings > 30))		Answer_UnknownParameter();
			else
			{
				LowLimit.GHTemp_LowerLimit = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;

			Value_Settings = atoi(Upper_Buffer);
			if((Value_Settings < 15) || (Value_Settings > 30))		Answer_UnknownParameter();
			else
			{
				UpLimit.GHTemp_UpperLimit = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;
		}

		if (strncmp(commandStr, GHRH, 5)==0)
		{
			strncpy(Lower_Buffer,commandStr+5,2);
			strncpy(Upper_Buffer,commandStr+8,2);

			Value_Settings = atoi(Lower_Buffer);
			if((Value_Settings < 20) || (Value_Settings > 90))		Answer_UnknownParameter();
			else
			{
				LowLimit.GHRh_LowerLimit = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;

			Value_Settings = atoi(Upper_Buffer);
			if((Value_Settings < 20) || (Value_Settings > 90))		Answer_UnknownParameter();
			else
			{
				UpLimit.GHRh_UpperLimit = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;
		}

		if (strncmp(commandStr, PH, 3)==0)
		{
			strncpy(Lower_Buffer,commandStr+3,1);
			strncpy(Upper_Buffer,commandStr+5,1);

			Value_Settings = atoi(Lower_Buffer);
			if((Value_Settings < 4) || (Value_Settings > 8))		Answer_UnknownParameter();
			else
			{
				LowLimit.Ph_LowerLimit = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;

			Value_Settings = atoi(Upper_Buffer);
			if((Value_Settings < 4) || (Value_Settings > 8))		Answer_UnknownParameter();
			else
			{
				UpLimit.Ph_UpperLimit = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;
		}

		if (strncmp(commandStr, PWMPUMP, 8)==0)
		{
			strncpy(Critical_Buffer,commandStr+8,2);
			Value_Settings = atoi(Critical_Buffer);
			if((Value_Settings < 10) || (Value_Settings > 100))		Answer_UnknownParameter();
			else
			{
				Filling_Factor.FillingFactor_PumpPWM = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;
		}
		if (strncmp(commandStr, LIGHT, 6)==0)
		{
			strncpy(Critical_Buffer,commandStr+6,4);
			Value_Settings = atoi(Critical_Buffer);
			if((Value_Settings < 0) || (Value_Settings > 3000))		Answer_UnknownParameter();
			else
			{
				CriticalParameters.Foto_CriticalValue = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;
		}
		if (strncmp(commandStr, CRIT_CO, 8)==0)
		{
			strncpy(Critical_Buffer,commandStr+8,4);
			Value_Settings = atoi(Critical_Buffer);
			if((Value_Settings < 100) || (Value_Settings > 2500))	Answer_UnknownParameter();
			else
			{
				CriticalParameters.CO_CriticalValue = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;
		}
		if (strncmp(commandStr, PIPE_DELAY, 11)==0)
		{
			strncpy(Critical_Buffer,commandStr+11,2);
			Value_Settings = atoi(Critical_Buffer);
			if((Value_Settings <= 0) || (Value_Settings > 20))		Answer_UnknownParameter();
			else
			{
				Delays.TimePipe_PumpDelay = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;
		}
		if (strncmp(commandStr, POOL_DELAY, 11)==0)
		{
			strncpy(Critical_Buffer,commandStr+11,2);
			Value_Settings = atoi(Critical_Buffer);
			if((Value_Settings <= 0) || (Value_Settings > 5))		Answer_UnknownParameter();
			else
			{
				Delays.TimePool_PumpDelay = Value_Settings;
				Answer_OK();
			}
			Value_Settings = 0;
		}

	}
}

void Start_Periphery(void)
{
	Write_LimitsFlag = FALSE;
	Pump_TimingFlag = TRUE;
	HAL_GPIO_WritePin(GPIOB, PWM_SWITCH_Pin, 1);
}
void Stop_Periphery(void)
{
	Write_LimitsFlag = FALSE;
	Pump_TimingFlag = FALSE;
	HAL_GPIO_WritePin(GPIOB, PWM_SWITCH_Pin, 0);

	Timer_Counter.DayDelay_Hour_Counter = 0;
	Timer_Counter.DayDelay_Minute_Counter = 0;
	Timer_Counter.PipePump_Minute_Counter = 0;
	Timer_Counter.PoolPump_Minute_Counter = 0;
	Timer_Counter.Second_Counter = 0;

	Filling_Factor.FillingFactor_LedPWM = 0;
	Filling_Factor.FillingFactor_PumpPWM = 0;
}
void Relay_Algorithm(void)
{
	if(valuesOfDS18B20.TEMP_DS18B20 > POOLTEMP_UPLIM)			HAL_GPIO_WritePin(GPIOB, CONTROL_Cooler_Pin, 1);
	else if(valuesOfDS18B20.TEMP_DS18B20 < POOLTEMP_LOWLIM)		HAL_GPIO_WritePin(GPIOB, CONTROL_WH_Pin, 1);
	else{
		HAL_GPIO_WritePin(GPIOB, CONTROL_WH_Pin, 0);
		HAL_GPIO_WritePin(GPIOB, CONTROL_Cooler_Pin, 0);
	}

	if(valuesOfDHT11.TEMP_DHT11 > UpLimit.GHTemp_UpperLimit)			HAL_GPIO_WritePin(GPIOB, CONTROL_FAN_Pin, 1);
	else if(valuesOfDHT11.TEMP_DHT11 < LowLimit.GHTemp_LowerLimit){
		HAL_GPIO_WritePin(GPIOB, CONTROL_FAN_Pin, 1);
		HAL_GPIO_WritePin(GPIOB, CONTROL_CH_HEATER_Pin, 1);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, CONTROL_FAN_Pin, 0);
		HAL_GPIO_WritePin(GPIOB, CONTROL_CH_HEATER_Pin, 0);
	}

	if(valuesOfDHT11.RH_DHT11 > UpLimit.GHRh_UpperLimit)		HAL_GPIO_WritePin(GPIOB, CONTROL_FAN_Pin, 1);
	else if(valuesOfDHT11.RH_DHT11 > UpLimit.GHRh_UpperLimit)	HAL_GPIO_WritePin(GPIOB, CONTROL_Hum_Pin, 1);
	else{
		HAL_GPIO_WritePin(GPIOB, CONTROL_Hum_Pin, 0);
		HAL_GPIO_WritePin(GPIOB, CONTROL_FAN_Pin, 0);
	}

	if(sensors_Data[0] > CriticalParameters.CO_CriticalValue)	HAL_GPIO_WritePin(GPIOB, CONTROL_FAN_Pin, 1);
	else	HAL_GPIO_WritePin(GPIOB, CONTROL_FAN_Pin, 0);
}

void PWM_Algorithm(void)
{
	if(!Night_Flag){
		if(sensors_Data[2] < CriticalParameters.Foto_CriticalValue)		Filling_Factor.FillingFactor_LedPWM++;
		if(sensors_Data[2] > CriticalParameters.Foto_CriticalValue)		Filling_Factor.FillingFactor_LedPWM--;
	}
	else	Filling_Factor.FillingFactor_LedPWM = 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
