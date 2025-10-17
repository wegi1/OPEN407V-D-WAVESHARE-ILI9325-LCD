/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "openx07v_c_lcd.h"
#include "fonts.h"
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
TIM_HandleTypeDef htim2;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint8_t idx[13];
uint8_t idy[13];
volatile u32 my_cntr ;
volatile u32 my_ctrl ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_FSMC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  BSP_LCD_Init();
  BSP_LCD_SetFont(&Font24);

  BSP_LCD_Clear(LCD_COLOR_BLACK);






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//*************************************************************************************

	  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	  HAL_TIM_Base_Stop_IT(&htim2);
	  TIM2->CR1 = 0;
	  TIM2->ARR = 83999999;
	  my_cntr = 0;
	  my_ctrl = 0;

	  TIM2->PSC = 0;
	  TIM2->CNT = 0;
	  HAL_TIM_Base_Start_IT(&htim2);


	  while(my_ctrl == 0) {
		  for(u16 i1 = 0; i1 < 240; i1++) {
			  for(u16 i2 = 0; i2 < 320; i2++) {
				  my_cntr++;
					LCD_REG = LCD_REG_32;
					LCD_RAM =  (239 - i1);
					LCD_REG = LCD_REG_33;
					LCD_RAM = i2;

					/* Prepare to write GRAM */
					LCD_REG = LCD_REG_34;

					/* Write 16-bit GRAM Reg */
					LCD_RAM = 0xF100;
			  }
		  }
	  }
	  HAL_TIM_Base_Stop_IT(&htim2);

	  BSP_LCD_DisplayStringAt(0, 85 , (uint8_t*) "DOTS / sec." , CENTER_MODE );
	  BSP_LCD_DisplayStringAt(0, 135, idx, CENTER_MODE);
	  HAL_Delay(2000);

	  BSP_LCD_Clear(LCD_COLOR_BLACK);

	  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN ;
	  __DSB();

	  LCD_REG = 0;
	  u32 dr_id = 0;
	  dr_id = LCD_RAM;

	  my_htoa32(idx, dr_id);

	  BSP_LCD_DisplayStringAt(0, 85, (uint8_t*) "LCD DRIVER", CENTER_MODE);
	  BSP_LCD_DisplayStringAt(0, 125, (uint8_t*) "     ILI" , LEFT_MODE);

	  BSP_LCD_DisplayStringAt(140, 125, &idx[6], LEFT_MODE);

	  HAL_Delay(1500);

	  TIM2->CR1 = 0;
	  TIM2->PSC = 0;
	  TIM2->CNT = 0;
	  TIM2->ARR = 0xffffffff;
	  TIM2->CR1 = 1;

	  u32 cntr = 0;

	  while(TIM2->CNT < 84000000) {
		  cntr++;
		  LCD_REG = 0x50;
		  LCD_RAM = 0;
		  LCD_REG = 0x51;
		  LCD_RAM = 239;
		  LCD_REG = 0x52;
		  LCD_RAM = 0;
		  LCD_REG = 0x53;
		  LCD_RAM = 319;
		  LCD_REG = 0x22;
		  for(u32 i = 0; i < 76800; i++ ) {
			  LCD_RAM = 0x1f;
		  }
	  }
	  TIM2->CR1 = 0;

	  my_utoa(idx, cntr);
	  BSP_LCD_DisplayStringAt(0, 95, idx, CENTER_MODE);
	  BSP_LCD_DisplayStringAt(0, 60, (uint8_t*) "CLS FRAMES", CENTER_MODE);
	  BSP_LCD_DisplayStringAt(0, 135, (uint8_t*) "PER SECOND", CENTER_MODE);


	HAL_Delay(1500);



//*************************************************************************************
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  BSP_LCD_Clear(LCD_COLOR_BLACK);
	  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);

	  BSP_LCD_DisplayStringAt(0, 95, (uint8_t *)"Copyright(c)", CENTER_MODE);
	  BSP_LCD_DisplayStringAt(0, 125, (uint8_t *)"Waveshare", CENTER_MODE);

	  HAL_Delay(1500);

	  BSP_LCD_Clear(LCD_COLOR_BLUE);
	  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	  BSP_LCD_DisplayStringAt(0, 95, (uint8_t *)"ILI9325 DRIVER", CENTER_MODE);
	  BSP_LCD_DisplayStringAt(0, 125, (uint8_t *)"3.2inch LCD Demo", CENTER_MODE);
	  HAL_Delay(1500);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 84000000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BL_PWM_Pin|LCD_BL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BL_PWM_Pin LCD_BL_Pin */
  GPIO_InitStruct.Pin = BL_PWM_Pin|LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  FSMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 29;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 7;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 7;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
