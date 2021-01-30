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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Structure to define an ADC-dependent sensor.
typedef struct {
    const uint8_t adc_index; // Defined by the ADC input index in STM32CubeMX.
    const ADC_HandleTypeDef* adc;
	const float counts_per_volt; // TODO: Should this be a smaller type?
	volatile uint8_t value_ready_for_read; // 1=ready, 0=incomplete
} ADCSensorTypeDef;

// Structure to define a simple two-state actuator.
typedef struct {
	const GPIO_TypeDef* port;
	const uint16_t pin_num; // Sized to match with HAL GPIO type definitions.
	const uint8_t normally_open; // 1=normally-open, 0=normally-closed
	uint8_t current_state; // 1=open, 0=closed
} ActuatorTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BLINKS_PER_INTERVAL 2
#define BLINK_PERIOD 500
#define BLINK_INTERVAL 1000
#define ADC1_VAL_SIZE 10
#define ADC_SENSORS_SIZE 1 // TODO: Change later once proper sensor requirements are in place.
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Flag communicating when there is a new value to read from the adc1_val buffer.
//volatile uint8_t adc1_value_ready = 0;
// DMA buffer containing the output of adc1.
volatile uint16_t adc1_val[ADC1_VAL_SIZE];
// Simple loop counter for the UART communication demo.
int count = 0;
// String buffer for the UART communication demo.
char msg[50];

// Defines the potentiometer ADC sensor used for HAL API testing.
ADCSensorTypeDef pot = {0, &hadc1, 0xFFF/3.3, 0};
// List of all defined ADC-dependent sensors.
ADCSensorTypeDef* adc_sensors[ADC_SENSORS_SIZE] = {&pot};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//	if(hadc->Instance == hadc1.Instance)
//		adc1_value_ready = 1;

	// For each defined sensor, check to see if it uses the same ADC which
	// triggered the callback. If so, update its struct with the new information.
	for(int i = 0; i < ADC_SENSORS_SIZE; i++) {
		if(adc_sensors[i]->adc->Instance == hadc->Instance) {
			adc_sensors[i]->value_ready_for_read = 1;
			// TODO: Implement call to control loop indicating new sensor information is available.
		}
	}
}

void ADCSensor_Trigger_Read() {
	// TODO: Add more ADC triggers when more ADCs are implemented.
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_val, ADC1_VAL_SIZE);
}

uint32_t ADCSensor_Read_Counts(ADCSensorTypeDef* adc_sensor) {
	// TODO: Should there be a way to return an error code if there isn't a new value ready?

	// Reset ready flag for next read operation.
	adc_sensor->value_ready_for_read = 0;
	// Return the correct value from the ADC buffer.
	return adc1_val[adc_sensor->adc_index];
}

void Actuator_Open(ActuatorTypeDef* actuator) {
	// If the actuator is normally open, write logic low. Else write logic high.
	HAL_GPIO_WritePin(actuator->port,
					  actuator->pin_num,
					  actuator->normally_open == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Actuator_Close(ActuatorTypeDef* actuator) {
	// If the actuator is normally open, write logic high. Else write logic low.
	HAL_GPIO_WritePin(actuator->port,
					  actuator->pin_num,
					  actuator->normally_open == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
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
  HAL_StatusTypeDef ret_status;
//  uint8_t uart_send_buffer[14] = {'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '!', '\r', '\n'};
//  memset(msg + strlen(msg), NULL, 30 - strlen(msg)); // https://stackoverflow.com/questions/33689274/how-to-fill-a-char-array-in-c/33689388
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
//  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_val, ADC_SIZE);
  ADCSensor_Trigger_Read();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
//	if(adc1_value_ready) {
	if(pot.value_ready_for_read) {
//	  	float voltage = (float)adc1_val[0]*(3.3/0xFFF);
		float voltage = ADCSensor_Read_Counts(&pot)/pot.counts_per_volt;

		sprintf(msg, "Hello World! %d ADC: %d.%d\r\n", count, (int)voltage, (int)(voltage * 100) % 100);
		ADCSensor_Trigger_Read();
//		adc1_value_ready = 0;
//		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_val, ADC_SIZE);
	} else {
		sprintf(msg, "Hello World! %d\r\n", count);
	}

//	ret_status = HAL_UART_Transmit(&huart1, (uint8_t*)&uart_send_buffer[0], strlen(uart_send_buffer), HAL_MAX_DELAY);
	ret_status = HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	for(int i = 0; i < BLINKS_PER_INTERVAL; i++) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(BLINK_PERIOD / 2);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_Delay(BLINK_PERIOD / 2);
	}
	HAL_Delay(BLINK_INTERVAL - BLINK_PERIOD * BLINKS_PER_INTERVAL);

//	ret_status = HAL_I2C_Slave_Receive(&hi2c1, (uint8_t*)&data_buffer, 1, HAL_MAX_DELAY);
//	if(ret_status != HAL_OK) {
//		// Error
//	} else {
//		uint8_t value = data_buffer[0];
//		uint8_t response = value > 0 ? -1 : 1;
//		ret_status = HAL_I2C_Slave_Transmit(&hi2c1, &response, 1, HAL_MAX_DELAY);
//		if(ret_status != HAL_OK) {
//			// Error
//		}
//	}

	count++;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_Board_LED_GPIO_Port, Green_Board_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Green_Board_LED_Pin */
  GPIO_InitStruct.Pin = Green_Board_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_Board_LED_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
