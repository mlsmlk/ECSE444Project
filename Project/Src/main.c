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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_qspi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLOCK_SIZE 65536
int blockMemoryIndex = 10;
int blockNum = 0;
int filledOnce = 0;
int eraseMode = 0;
int beginUART = 0;
int readblockMemoryIndex = 10;
int readBlockNum = 0;
int blocksRead = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_QUADSPI_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize I2C sensors */
  BSP_ACCELERO_Init();
//  BSP_QSPI_Erase_Chip(); //erase chip for first run
  for(int i =0; i <= 36; i++){
	  BSP_QSPI_Erase_Block(i*BLOCK_SIZE); //erase the first block that will hold the sound
  }
  /* Set low power mode for accelerometer and magnetometer */
  BSP_ACCELERO_LowPower(1);

  /* Initialize QSPI */
  BSP_QSPI_Init();

  //store the sound wave in QSPI
  uint8_t sineWave[32]; //TODO: ask team if this is the wave to store
  float radian_1 = 0;
  for (int i = 0; i < 32; i++) {
	  sineWave[i] = 100 * arm_sin_f32(radian_1) + 100;
	  radian_1 += PI/2;
  }
  BSP_QSPI_Write(sineWave, 0, 32); //write the sound to the block

  int16_t accelero_XYZ[3];
  char accelero_XYZ_buffer[100];
  int16_t prev_accelero_XYZ[3];
  char max_accelero_XYZ_buffer[100];
  BSP_ACCELERO_AccGetXYZ(prev_accelero_XYZ);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  printf("test\n");

	  /*Read Acceleration Value*/
	  BSP_ACCELERO_AccGetXYZ(accelero_XYZ);

	  //Cartesian Distance
	  double x = pow(accelero_XYZ[0]-prev_accelero_XYZ[0],2);
	  double y = pow(accelero_XYZ[1]-prev_accelero_XYZ[1],2);
	  double z = pow(accelero_XYZ[2]-prev_accelero_XYZ[2],2);
	  double sum = x+y+z;
	  double dist = sqrt(sum);

	  //Update previous point
	  prev_accelero_XYZ[0] = accelero_XYZ[0];
	  prev_accelero_XYZ[1] = accelero_XYZ[1];
	  prev_accelero_XYZ[2] = accelero_XYZ[2];

	  int x1 = accelero_XYZ[0];
	  int x2 = accelero_XYZ[1];
	  int x3 = accelero_XYZ[2];


	  /*Debugging Purposes*/
	  sprintf(accelero_XYZ_buffer, "\nAcceleration: X:%d Y:%d Z:%d\n",(int) accelero_XYZ[0],(int) accelero_XYZ[1],(int)accelero_XYZ[2]);
	  HAL_UART_Transmit(&huart1, accelero_XYZ_buffer, 100, 30000);


	  /*Convert in terms of g*/
	  double dist_g = dist/9.81;

	  /*Debugging if the peak values are correct in terms of g*/
	  sprintf(max_accelero_XYZ_buffer, "\nMagnitude: %d g\n", (int) dist);
	  HAL_UART_Transmit(&huart1, max_accelero_XYZ_buffer, 100, 30000);

	  if(dist_g > 300) {

		int magnitude = 0;

		if (30 <= dist_g && dist_g < 39){
			magnitude = 4;
		} else if(39 <= dist_g && dist_g < 92){
			magnitude = 5;
		} else if (92 <= dist_g && dist_g < 180) {
			magnitude = 6;
		} else if (180 <= dist_g && dist_g < 340){
			magnitude = 7;
		} else if (340 <= dist_g && dist_g < 650){
			magnitude = 8;
		} else if (650 <= dist_g && dist_g < 1240){
			magnitude = 9;
		} else if (dist_g >= 1240){
			magnitude = 10;
		}


		//TODO: We should store in flash something like : PGA: X: %d g Y:%d g Z:%d g, Mag:%d, Time:time(NULL)


		if(magnitude>5){

			//write acceleration values and magnitude to respective blocks
			BSP_QSPI_Write(accelero_XYZ[0], (blockNum+1)*BLOCK_SIZE + blockMemoryIndex, 2);
			BSP_QSPI_Write(accelero_XYZ[1], (blockNum+2)*BLOCK_SIZE + blockMemoryIndex, 2);
			BSP_QSPI_Write(accelero_XYZ[2], (blockNum+3)*BLOCK_SIZE + blockMemoryIndex, 2);
			BSP_QSPI_Write(magnitude, (blockNum+4)*BLOCK_SIZE + blockMemoryIndex, 2);


			blockMemoryIndex += 2;
			if(blockMemoryIndex >= BLOCK_SIZE-8){ //last write in the block at address BlockSize-10
				blockMemoryIndex = 10; //first write in the block at address 10, if done from 0 to blocksize causes some issues at the borders of the block
				blockNum += 4;
			}

			if(blockNum > 32){ //provides 8 blocks of memory for each x y z and
				blockNum = 0;
				filledOnce = 1;
				eraseMode = 1;
			}

			if(filledOnce == 1 && eraseMode == 1){
				//erase the ones of the next blockNum
				BSP_QSPI_Erase_Block(BLOCK_SIZE*(blockNum +1)); //erase x block
				BSP_QSPI_Erase_Block(BLOCK_SIZE*(blockNum +2)); //erase y block
				BSP_QSPI_Erase_Block(BLOCK_SIZE*(blockNum +3)); //erase z block
				BSP_QSPI_Erase_Block(BLOCK_SIZE*(blockNum +4)); //erase magnitude block

			}

			uint8_t sine_1[32];
			BSP_QSPI_Read(sine_1, 0, 32); //get the soundwave stored in the first block of the QSPI
			HAL_TIM_Base_Start(&htim2);
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) sine_1, 32, DAC_ALIGN_8B_R);
		}
		magnitude = 0;


		//transmit if button pressed
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_SET){				//if button pressed
			while(1){
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
					beginUART = 1;
				}
			}
		}

		while(beginUART) { //want to display the values in chronological order obtained
			uint16_t spiX, spiY, spiZ, spiMag;

			if(filledOnce == 1 && blocksRead == 0){ //if was filled at least once oldest value is the 4 blocks ahead of the ones being written
				readBlockNum = blockNum +4;
				if(readBlockNum > 32) readBlockNum = 0; //reset the number if we are at the last set of 4 blocks
				readblockMemoryIndex = 10;
			}

			//read values
			BSP_QSPI_Read(&spiX, (readBlockNum+1)*BLOCK_SIZE + readblockMemoryIndex, 2);
			BSP_QSPI_Read(&spiY, (readBlockNum+1)*BLOCK_SIZE + readblockMemoryIndex, 2);
			BSP_QSPI_Read(&spiZ, (readBlockNum+1)*BLOCK_SIZE + readblockMemoryIndex, 2);
			BSP_QSPI_Read(&spiMag, (readBlockNum+1)*BLOCK_SIZE + readblockMemoryIndex, 2);

			//print to uart
			char uartTransmit[100];

			sprintf(uartTransmit, "\nAcceleration: X:%d Y:%d Z:%d; Magnitude: %d\n",(int) spiX,(int) spiY,(int)spiZ, (int)spiMag);
			HAL_UART_Transmit(&huart1, accelero_XYZ_buffer, 100, 30000);

			readblockMemoryIndex += 2;

			if(readblockMemoryIndex >= BLOCK_SIZE-8){ //last write in the block at address BlockSize-10
				readblockMemoryIndex = 10; //first write in the block at address 10, if done from 0 to blocksize causes some issues at the borders of the block
				readBlockNum += 4;
				blocksRead++;
			}

			if(blocksRead == 8){ //provides 8 blocks of memory for each x y z and
				readBlockNum = 0;
				char endStr[100];
				sprintf(endStr, "\nDATA TRANSMISSION FROM QSPI COMPLETE\n");
				HAL_UART_Transmit(&huart1, endStr, 100, 30000);
				readblockMemoryIndex = 10;
				readBlockNum = 0;
				blocksRead = 0;
				break;
			}



			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_SET){				//if button pressed stop transmission
				while(1){
					if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
						beginUART = 0;
						readblockMemoryIndex = 10; //reset the transmission values;
						readBlockNum = 0;
						blocksRead = 0;
						break;
					}
				}
			}

		}
		HAL_Delay(50); //20Hz
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c2.Init.Timing = 0x10909CEC;
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
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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
  htim2.Init.Period = 1814;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	}
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
