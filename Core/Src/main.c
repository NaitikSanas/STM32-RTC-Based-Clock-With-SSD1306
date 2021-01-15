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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "fonts.h"
#include "ssd1306.h"
#include "print.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId CounterHandle;
osThreadId OLED_DriverHandle;
osThreadId led_onHandle;
osThreadId led_offHandle;
osMessageQId Cnt_QueueHandle;
osSemaphoreId semHandle;
/* USER CODE BEGIN PV */
#include <stdbool.h>
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void const * argument);
void CntHandle(void const * argument);
void OLED_Handler(void const * argument);
void ledOn(void const * argument);
void ledOFF(void const * argument);

/* USER CODE BEGIN PFP */
static uint8_t int2bcd(uint8_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RTC_DateTypeDef gDate;
RTC_TimeTypeDef gTime;
static void clock(void);
bool toggle = true;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	SSD1306_Init();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of sem */
  osSemaphoreDef(sem);
  semHandle = osSemaphoreCreate(osSemaphore(sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Cnt_Queue */
  osMessageQDef(Cnt_Queue, 2, uint16_t);
  Cnt_QueueHandle = osMessageCreate(osMessageQ(Cnt_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Counter */
  osThreadDef(Counter, CntHandle, osPriorityNormal, 0, 128);
  CounterHandle = osThreadCreate(osThread(Counter), NULL);

  /* definition and creation of OLED_Driver */
  osThreadDef(OLED_Driver, OLED_Handler, osPriorityAboveNormal, 0, 128);
  OLED_DriverHandle = osThreadCreate(osThread(OLED_Driver), NULL);

  /* definition and creation of led_on */
  osThreadDef(led_on, ledOn, osPriorityNormal, 0, 128);
  led_onHandle = osThreadCreate(osThread(led_on), NULL);

  /* definition and creation of led_off */
  osThreadDef(led_off, ledOFF, osPriorityNormal, 0, 128);
  led_offHandle = osThreadCreate(osThread(led_off), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010061A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00300F38;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x14;
  sTime.Minutes = 0x50;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 0x21;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static uint8_t int2bcd(uint8_t value){

	uint8_t shift = 0;
	uint8_t bcd = 0;

	  while (value > 0) {
	        bcd |= (value % 10) << (shift++ << 2);
	        value /= 10;
	     }
	return bcd;

}			 
static void clock(){
	 /* Get the RTC current Time */
	  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	 /* Get the RTC current Date */
	  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
				
		
	  SSD1306_Clear();
	  SSD1306_GotoXY (15,0);
		uint8_t sbcd = int2bcd(gTime.Seconds);
	  uint8_t mbcd = int2bcd(gTime.Minutes);
	  uint8_t hbcd = int2bcd(gTime.Hours);
	  //SSD1306_Puts (sec, &Font_11x18, 1);
	  SSD1306_Putc ( (char)(((hbcd&0xF0)>>4) + 48), &Font_16x26, (SSD1306_COLOR_t)1);
	  SSD1306_Putc ( (char)((hbcd&(uint8_t)0x0F)+(uint8_t)48), &Font_16x26, (SSD1306_COLOR_t)1);

	  if(toggle) SSD1306_Putc ( ':', &Font_16x26, (SSD1306_COLOR_t)1);
	  else SSD1306_Putc ( ' ', &Font_16x26, (SSD1306_COLOR_t)1);

	  SSD1306_Putc ( (char)(((mbcd&0xF0)>>4) + 48), &Font_16x26, (SSD1306_COLOR_t)1);
	  SSD1306_Putc ( (char)((mbcd&0x0F)+48), &Font_16x26, (SSD1306_COLOR_t)1);
	  
		SSD1306_Putc ( ':', &Font_7x10,(SSD1306_COLOR_t)1);
    SSD1306_Putc ( (char)(((sbcd&0xF0)>>4) + 48), &Font_7x10, (SSD1306_COLOR_t)1);
    SSD1306_Putc ( (char)((sbcd&0x0F)+48), &Font_7x10,(SSD1306_COLOR_t)1);

	  SSD1306_GotoXY(15,25);
	  SSD1306_Putc ( (char)(((int2bcd(gDate.Date)&0xF0)>>4) + 48), &Font_11x18, (SSD1306_COLOR_t)1);
	  SSD1306_Putc ( (char)(( int2bcd(gDate.Date)&0x0F)+48), &Font_11x18, (SSD1306_COLOR_t)1);
	  SSD1306_Putc ( '/', &Font_11x18, (SSD1306_COLOR_t)1);
	  
		SSD1306_Putc ( (char)(((int2bcd(gDate.Month)&0xF0)>>4) + 48), &Font_11x18, (SSD1306_COLOR_t)1);
	  SSD1306_Putc ( (char)((int2bcd(gDate.Month)&0x0F)+48), &Font_11x18, (SSD1306_COLOR_t)1);
	  SSD1306_Putc ( '/', &Font_11x18, (SSD1306_COLOR_t)1);
	  
		SSD1306_Putc ( (char)(((int2bcd(gDate.Year)&0xF0)>>4) + 48), &Font_11x18, (SSD1306_COLOR_t)1);
	  SSD1306_Putc ( (char)((int2bcd(gDate.Year)&0x0F)+48), &Font_11x18, (SSD1306_COLOR_t)1);


	  if (gTime.Hours >= 5 && gTime.Hours < 12){
		  SSD1306_GotoXY(5,50);
		  SSD1306_Puts("Good Morning :)",&Font_7x10,(SSD1306_COLOR_t)1);
	  }
	  if (gTime.Hours >= 12 && gTime.Hours <= 16) {
		  SSD1306_GotoXY(5,50);
		  SSD1306_Puts("Good Afternoon :)",&Font_7x10,(SSD1306_COLOR_t)1);
	  }
	  if (gTime.Hours > 16 && gTime.Hours < 20   ) {
		  SSD1306_GotoXY(5,50);
		  SSD1306_Puts("Good Evening :)",&Font_7x10,(SSD1306_COLOR_t)1);
	  }
	  if (gTime.Hours >= 20  && gTime.Hours <= 4  ) {
		  SSD1306_GotoXY(5,50);
		  SSD1306_Puts("Good Night :)",&Font_7x10,(SSD1306_COLOR_t)1);
	  }

	  toggle = !toggle;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_CntHandle */
/**
* @brief Function implementing the Counter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CntHandle */
void CntHandle(void const * argument)
{
  /* USER CODE BEGIN CntHandle */
	uint8_t cnt = 0;
  /* Infinite loop */
  for(;;)
  {
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == GPIO_PIN_RESET){
			while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == GPIO_PIN_RESET);
			cnt++;
			osMessagePut(Cnt_QueueHandle,cnt,osWaitForever);
			osDelay(100);
		}
		osDelay(100);
  }
  /* USER CODE END CntHandle */
}

/* USER CODE BEGIN Header_OLED_Handler */
/**
* @brief Function implementing the OLED_Driver thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLED_Handler */
void OLED_Handler(void const * argument)
{
  /* USER CODE BEGIN OLED_Handler */
	osEvent evt;
	uint8_t counter = 0;
  /* Infinite loop */
  for(;;)
  {
		evt = osMessageGet(Cnt_QueueHandle,1);
		HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	 /* Get the RTC current Date */
	  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
				
		if(evt.status == osEventMessage) counter = evt.value.v;
		printMsg("%d : %d : %d | counter: %d \r \n", gTime.Hours,gTime.Minutes,gTime.Seconds, counter);
		clock();
		SSD1306_GotoXY(1,1);
	  SSD1306_Putc ((char)(((counter&0xF0)>>4) + 48), &Font_7x10, (SSD1306_COLOR_t)1);
	  SSD1306_Putc ((char)((counter&0x0F)+(48)), &Font_7x10, (SSD1306_COLOR_t)1);
    SSD1306_UpdateScreen();
    osDelay(1000);
  }
  /* USER CODE END OLED_Handler */
}

/* USER CODE BEGIN Header_ledOn */
/**
* @brief Function implementing the led_on thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledOn */
void ledOn(void const * argument)
{
  /* USER CODE BEGIN ledOn */
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake(semHandle,osWaitForever);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		osDelay(1000);
		xSemaphoreGive(semHandle);		
    
		
  }
  /* USER CODE END ledOn */
}

/* USER CODE BEGIN Header_ledOFF */
/**
* @brief Function implementing the led_off thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledOFF */
void ledOFF(void const * argument)
{
  /* USER CODE BEGIN ledOFF */
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(semHandle,osWaitForever);
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		osDelay(1000);
		xSemaphoreGive(semHandle);
		
  }
  /* USER CODE END ledOFF */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
