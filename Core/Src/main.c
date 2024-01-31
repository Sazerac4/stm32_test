/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "sine.h"
#include "median_filter.h"
#include "MedianFilter.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define min(a,b) (a<b ? a : b)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
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
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  static struct MEDFILT_Node medfilt_wndw7[7];
  static struct MEDFILT_Handle hmedfilt7={.window_len=7, .window=medfilt_wndw7};
  static struct MEDFILT_Node medfilt_wndw15[15];
  static struct MEDFILT_Handle hmedfilt15={.window_len=15, .window=medfilt_wndw15};
  static struct MEDFILT_Node medfilt_wndw31[31];
  static struct MEDFILT_Handle hmedfilt31={.window_len=31 , .window=medfilt_wndw31};

  MEDFILT_Init(&hmedfilt7);  
  MEDFILT_Init(&hmedfilt15);  
  MEDFILT_Init(&hmedfilt31);  



  HAL_TIM_Base_Start(&htim7);

  static uint32_t time_start7, time_end7;
  static uint32_t time_start15, time_end15;
  static uint32_t time_start31, time_end31;
  static uint32_t sample_cnt = 0;
  static uint32_t filtered_sample7 = 0;
  static uint32_t filtered_sample15 = 0;
  static uint32_t filtered_sample31 = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t sample = sinewave[sample_cnt%256];

    time_start7 = __HAL_TIM_GET_COUNTER(&htim7);
    filtered_sample7 = MEDFILT_Insert(&hmedfilt7,sample);
    time_end7 = __HAL_TIM_GET_COUNTER(&htim7);

    time_start15 = __HAL_TIM_GET_COUNTER(&htim7);
    filtered_sample15 = MEDFILT_Insert(&hmedfilt15,sample);
    time_end15 = __HAL_TIM_GET_COUNTER(&htim7);

    time_start31 = __HAL_TIM_GET_COUNTER(&htim7);
    filtered_sample31 = MEDFILT_Insert(&hmedfilt31,sample);
    time_end31 = __HAL_TIM_GET_COUNTER(&htim7);

    char buf[50];
    uint32_t cycles7 = min(((uint16_t)(-1U))-abs(time_end7-time_start7),abs(time_end7-time_start7));
    uint32_t cycles15 = min(((uint16_t)(-1U))-abs(time_end15-time_start15),abs(time_end15-time_start15));
    uint32_t cycles31 = min(((uint16_t)(-1U))-abs(time_end31-time_start31),abs(time_end31-time_start31));
    sprintf(buf,"%u,%u,%u,%u,%u,%u,%u,%u\n\r",(uint8_t)sample_cnt%256,sample,filtered_sample7,filtered_sample15,filtered_sample31,cycles7,cycles15,cycles31);
    HAL_UART_Transmit(&huart2,(uint8_t *)buf,strlen(buf),100U);
    HAL_Delay(100);

    sample_cnt++;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
/*
* arm-none-eabi-gcc build/main.o build/MedianFilter.o build/stm32f4xx_it.o
* build/stm32f4xx_hal_msp.o build/stm32f4xx_hal_tim.o
* build/stm32f4xx_hal_tim_ex.o build/stm32f4xx_hal_uart.o
* build/stm32f4xx_hal_rcc.o build/stm32f4xx_hal_rcc_ex.o
* build/stm32f4xx_hal_flash.o build/stm32f4xx_hal_flash_ex.o
* build/stm32f4xx_hal_flash_ramfunc.o build/stm32f4xx_hal_gpio.o
* build/stm32f4xx_hal_dma_ex.o build/stm32f4xx_hal_dma.o
* build/stm32f4xx_hal_pwr.o build/stm32f4xx_hal_pwr_ex.o
* build/stm32f4xx_hal_cortex.o build/stm32f4xx_hal.o build/stm32f4xx_hal_exti.o
* build/system_stm32f4xx.o build/startup_stm32f446xx.o -mcpu=cortex-m4 -mthumb
* -mfpu=fpv4-sp-d16 -mfloat-abi=hard -specs=nano.specs -TSTM32F446RETx_FLASH.ld
*  -lc -lm -lnosys  -Wl,-Map=build/test.map,--cref -Wl,--gc-sections -o
*  build/test.elf
/usr/lib/gcc/arm-none-eabi/13.2.0/../../../../arm-none-eabi/bin/ld: /usr/lib/gcc/arm-none-eabi/13.2.0/../../../../arm-none-eabi/lib/thumb/v7e-m+fp/hard/libc_nano.a(libc_a-closer.o): in function `_close_r':
/build/arm-none-eabi-newlib/src/build-nano/arm-none-eabi/thumb/v7e-m+fp/hard/newlib/../../../../../../newlib-4.3.0.20230120/newlib/libc/reent/closer.c:47:(.text._close_r+0xc): warning: _close is not implemented and will always fail
/usr/lib/gcc/arm-none-eabi/13.2.0/../../../../arm-none-eabi/bin/ld: /usr/lib/gcc/arm-none-eabi/13.2.0/../../../../arm-none-eabi/lib/thumb/v7e-m+fp/hard/libc_nano.a(libc_a-lseekr.o): in function `_lseek_r':
/build/arm-none-eabi-newlib/src/build-nano/arm-none-eabi/thumb/v7e-m+fp/hard/newlib/../../../../../../newlib-4.3.0.20230120/newlib/libc/reent/lseekr.c:49:(.text._lseek_r+0x10): warning: _lseek is not implemented and will always fail
/usr/lib/gcc/arm-none-eabi/13.2.0/../../../../arm-none-eabi/bin/ld: /usr/lib/gcc/arm-none-eabi/13.2.0/../../../../arm-none-eabi/lib/thumb/v7e-m+fp/hard/libc_nano.a(libc_a-readr.o): in function `_read_r':
/build/arm-none-eabi-newlib/src/build-nano/arm-none-eabi/thumb/v7e-m+fp/hard/newlib/../../../../../../newlib-4.3.0.20230120/newlib/libc/reent/readr.c:49:(.text._read_r+0x10): warning: _read is not implemented and will always fail
/usr/lib/gcc/arm-none-eabi/13.2.0/../../../../arm-none-eabi/bin/ld: /usr/lib/gcc/arm-none-eabi/13.2.0/../../../../arm-none-eabi/lib/thumb/v7e-m+fp/hard/libc_nano.a(libc_a-writer.o): in function `_write_r':
/build/arm-none-eabi-newlib/src/build-nano/arm-none-eabi/thumb/v7e-m+fp/hard/newlib/../../../../../../newlib-4.3.0.20230120/newlib/libc/reent/writer.c:49:(.text._write_r+0x10): warning: _write is not implemented and will always fail
/usr/lib/gcc/arm-none-eabi/13.2.0/../../../../arm-none-eabi/bin/ld: warning: build/test.elf has a LOAD segment with RWX permissions
*/
