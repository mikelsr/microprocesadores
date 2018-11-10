
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/** VARIABLES **/

/* flags */
volatile uint8_t alarm_a=0;
volatile uint32_t AR_DEBUG, DR_DEBUG, TR_DEBUG, tickCount;

/* consts */
const uint8_t	SHIFT_YEAR=16,	SHIFT_MONTH=8,	SHIFT_DAY=0;
const uint8_t	SHIFT_HOUR=16,	SHIFT_MIN=8,		SHIFT_SEC=0;
const uint8_t LEN_DATE = 17, LEN_ALARM=8; // length of date and alarm responses
const uint32_t CHECK_PERIOD=60*1000; // 1 min between updates

/* buffers */
volatile char recv_buff_alarm[LEN_ALARM];
volatile char recv_buff_date[LEN_DATE];

/* variables */
uint32_t AR, DR, TR, timeout=10000;

char ip[]="192.168.1.70", port[]="8080";
char ssid[]="SanPedroTxurru", passwd[]="655ama739386";

char at_cwjap[50], at_cipstart[50];
char at_rst[] = "AT+RST";

/** FUNCTIONS **/

/*
 * recv will return 0 if the message is successfully received, 1 otherwise
 */
uint8_t at_recv(char expected[])
{
	char recv[1];
	int len=strlen(expected);
	int count=0;

	while (1) {
		HAL_StatusTypeDef status = HAL_UART_Receive(&huart6, (uint8_t *)recv, 1, timeout);
		if (status != HAL_OK) return 1;
		
		// character matches expected character
		if (recv[0] == expected[count]) {
			// last character
			if (++count == len) return 0;
		// characters don't match
		} else {
			// reset count
			count = 0;
			// character is first character of expected message
			if (recv[0] == expected[0]) count = 1;
		}
	}
}

/*
 * send appends "\r\n" to a command and calls HAL_UART_Transmit
 * returns 0 if OK 1 otherwise
 */
uint8_t at_send(char cmd[])
{
	char payload[strlen(cmd)+2];
	sprintf(payload, "%s\r\n", cmd);
	
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, (uint8_t *)payload, strlen(payload), timeout);
	if (status != HAL_OK) return 1;
	return 0;
}

/*
 * handler of alarm A
 */
void h_alarm_a()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_Delay(5000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	alarm_a = 0;
}

/*
 * init_wifi resets the ESP8266 module and connects it to the AP
 */
void init_wifi(void) {
	at_send(at_rst);
	at_recv("ready\r\n");

	at_send(at_cwjap);
	at_recv("OK\r\n");
}

/*
 * init_at_cmds initializes commands with variable content
 */
void init_at_cmds(void)
{
	sprintf(at_cwjap, "AT+CWJAP=\"%s\",\"%s\"", ssid, passwd);
	sprintf(at_cipstart, "AT+CIPSTART=\"TCP\",\"%s\",%s", ip, port);
}

/*
 * weekday returns the weekday of a date
 */
uint8_t weekday(uint8_t year, uint8_t month, uint8_t day)
{
	return (day += month < 3 ? (year-1) : year - 2, 23*month/9 + day + 4 + year/4- year/100 + year/400)%7;
}

uint8_t req_alarm(uint32_t *AR)
{
	/* start connection */
	at_send(at_cipstart);
	at_recv("OK\r\n");

	/* send request */
	char at_cipsend[] = "AT+CIPSEND=1\r\n";
	char at_cipsend_content[] = "A";
	at_send(at_cipsend);
	at_recv("\r\n>");
	HAL_UART_Transmit(&huart6, (uint8_t *)at_cipsend_content, strlen(at_cipsend_content), timeout);
	
	/* receive alarm time */
	char delim[6];
	sprintf(delim, "IPD,%d:", LEN_ALARM);
	at_recv(delim);
	HAL_UART_Receive(&huart6, (uint8_t *)recv_buff_alarm, LEN_ALARM, timeout);
	
	/* parse alarm time */
	
	/* ascii to value */
	for (int i=0; i<LEN_ALARM; i++) {
		recv_buff_alarm[i] = recv_buff_alarm[i] - 0x30;
	}
	
	// hour
	*AR =		recv_buff_alarm[0]	<< (SHIFT_HOUR+4)	| recv_buff_alarm[1] << SHIFT_HOUR;
	// minute
	*AR |=	recv_buff_alarm[3]	<< (SHIFT_MIN+4)	| recv_buff_alarm[4] << SHIFT_MIN;
	// second
	*AR |=	recv_buff_alarm[6]	<< (SHIFT_SEC+4)	| recv_buff_alarm[7] << SHIFT_SEC;
	
	AR_DEBUG = *AR;
	
	return 0;
}


uint8_t req_datetime(uint32_t *DR, uint32_t *TR)
{
	/* start connection */
	at_send(at_cipstart);
	at_recv("OK\r\n");

	/* send request */
	char at_cipsend[] = "AT+CIPSEND=1\r\n";
	char at_cipsend_content[] = "D";
	at_send(at_cipsend);
	at_recv("\r\n>");
	HAL_UART_Transmit(&huart6, (uint8_t *)at_cipsend_content, strlen(at_cipsend_content), timeout);
	
	/* receive datetime */
	char delim[7];
	sprintf(delim, "IPD,%d:", LEN_DATE);
	at_recv(delim);
	HAL_UART_Receive(&huart6, (uint8_t *)recv_buff_date, LEN_DATE, timeout);
	
	/* parse datetime */
	
	/* ascii to value */
	for (int i=0; i<LEN_DATE; i++) {
		recv_buff_date[i] = recv_buff_date[i] - 0x30;
	}
	
	// year
	*DR =		recv_buff_date[0] << (SHIFT_YEAR+4)		| recv_buff_date[1] << SHIFT_YEAR;
	// month
	*DR |=	recv_buff_date[3] << (SHIFT_MONTH+4)	| recv_buff_date[4] << SHIFT_MONTH;
	// day
	*DR |=	recv_buff_date[6] << (SHIFT_DAY+4)		| recv_buff_date[7] << SHIFT_DAY;
	
	// hour
	*TR =		recv_buff_date[9]		<< (SHIFT_HOUR+4)	| recv_buff_date[10] << SHIFT_HOUR;
	// minute
	*TR |=	recv_buff_date[12]	<< (SHIFT_MIN+4)	| recv_buff_date[13] << SHIFT_MIN;
	// second
	*TR |=	recv_buff_date[15]	<< (SHIFT_SEC+4)	| recv_buff_date[16] << SHIFT_SEC;
	
	DR_DEBUG = *DR;
	TR_DEBUG = *TR;
	
	return 0;
}

/*
 * set_alarm configures the alarm to trigger every day at the specified hour
 */
uint8_t set_alarm(uint32_t TR)
{
	RTC_AlarmTypeDef sAlarm;
	uint8_t hour, minute, second;

	hour 		= (TR & 0xFF0000)	>> SHIFT_HOUR;
	minute	= (TR & 0xFF00)		>> SHIFT_MIN;
	second	= (TR & 0xFF) 		>> SHIFT_SEC;

	sAlarm.AlarmTime.Hours = hour;
	sAlarm.AlarmTime.Minutes = minute;
	sAlarm.AlarmTime.Seconds = second;
	sAlarm.AlarmTime.SubSeconds = 0x0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;

	sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_A;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK) {
		return 1;
	}
	return 0;
}

/*
 * set_datetime sets the RTC date and time of the board
 */
uint8_t set_datetime(uint32_t DR, uint32_t TR)
{
	RTC_DateTypeDef sDate;
	RTC_TimeTypeDef sTime;
	uint8_t year, month, day, hour, minute, second;

	year	= (DR & 0xFF0000)	>> SHIFT_YEAR;
	month	= (DR & 0xFF00)		>> SHIFT_MONTH;
	day		= (DR & 0xFF)			>> SHIFT_DAY;

	hour		= (TR & 0xFF0000)	>> SHIFT_HOUR;
	minute	= (TR & 0xFF00)		>> SHIFT_MIN;
	second	= (TR & 0xFF)			>> SHIFT_SEC;

	/* initialize RTC*/
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		return 1;
	}

	/* set time */
	sTime.Hours = hour;
	sTime.Minutes = minute;
	sTime.Seconds = second;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		return 1;
	}

	/* set date */ 
	sDate.WeekDay = weekday(year, month, day);
	sDate.Year = year;
	sDate.Month = month;
	sDate.Date = day;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
		return 1;
	}
	return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	init_at_cmds();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	HAL_SYSTICK_Config(84000);
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

	init_wifi();
	
	req_datetime(&DR, &TR);
	set_datetime(DR, TR);
	
	req_alarm(&AR);
	set_alarm(AR);
	tickCount = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		while(tickCount < CHECK_PERIOD && !alarm_a);
		
		if (alarm_a) {
			h_alarm_a();
		}
		
		if (tickCount < CHECK_PERIOD) {
			req_datetime(&DR, &TR);
			set_datetime(DR, TR);
			
			req_alarm(&AR);
			set_alarm(AR);
			tickCount = 0;
		}
  /* USER CODE END WHILE */
	
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
void MX_RTC_Init(void)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	RTC_AlarmTypeDef sAlarm;

	/**Initialize RTC Only
	*/
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initialize RTC and set the Time and Date
	*/
	sTime.Hours = 0x12;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
					_Error_Handler(__FILE__, __LINE__);
	}

	/**Enable the Alarm A
	*/
	sAlarm.AlarmTime.Hours = 0x00;
	sAlarm.AlarmTime.Minutes = 0x00;
	sAlarm.AlarmTime.Seconds = 0x00;
	sAlarm.AlarmTime.SubSeconds = 0x0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	// sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_A;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Enable the Alarm B
	*/
	// sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_B;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
