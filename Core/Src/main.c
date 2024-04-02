/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
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
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t TxData[3];
uint32_t bit = 0;
uint32_t data = 0;

typedef enum{
  BUTTON_1,
  BUTTON_2,
  BUTTON_3,
  BUTTON_4,
  //could track up to 32 buttons
  NONE_B
}button_state;

button_state button = NONE_B;
GPIO_PinState state;

char key[] = "1101";	//edit to implement with any size key? (crc code only works for 4 numbered keys)
char data_str[40];
char remain[5];
char appended_data[40];
char result[5];
char send_data[40];
char tmp[10];

/**
 * sets specific button pressed
 */
void bitmask_set(uint32_t bit_position){
	bit |= (1 << bit_position);
}

/**
 * clears specific button pressed
 */
void bitmask_clear(uint32_t bit_position){
	 bit &= ~(1 << bit_position);
}

/**
 * checks if specific button is pressed
 */
uint8_t bitmask_check(uint32_t bit_position){
	if(bit & (1 << bit_position)){
		return 1;
	}else{
		return 0;
	}
}

/**
 * XOR logic used to divide data by key
 */
void xor(char* str1, char* str2){
	for(int i = 0; i < 5; i++){
		if(str1[i] == str2[i]){
			result[i] = '0';			//if bits are same, XOR is 0
		}else{
			result[i] = '1';			//if bits are different, XOR is 1
		}
	}
	result[4] = '\0';
}

/**
 * converts uint32 to string
 */
void toStr(){
	uint32_t temp = data;
	for(int i = 0; i < 32; i++){
		if(temp & 1){
			data_str[i] = '1';
		}else{
			data_str[i] = '0';
		}
		temp = temp >> 1;
	}
}

/**
 * divides data by key to get remainder
 *
 * takes 4 bits at a time and XORs them until 4 bit remainder is left
 */
void division(){
	int dividend_len = strlen(appended_data);
	int xor_bits = 4;

	strncpy(tmp, appended_data, xor_bits);

	while(xor_bits < dividend_len){
		if(tmp[0] == '1'){		//if leftmost bit is 1, perform xor with key
			xor(key, tmp);
			strncpy(tmp, result+1, 4);
			strcat(tmp, &appended_data[xor_bits]);
		}else{					//if leftmost bit is 0, perform xor with string of zeros
			xor("0000", tmp);
			strncpy(tmp, result+1, 4);
			strcat(tmp, &appended_data[xor_bits]);
		}
		xor_bits++;
	}

	if(tmp[0] == '1'){
		xor(key, tmp);
		strcpy(tmp, result);
	}else{
		xor("0000", tmp);
		strcpy(tmp, result);
	}
	strcpy(remain, tmp);
}

/**
 * reverses string
 */
char* str_rev(char* str){
	int len = strlen(str);
	for(int i = 0, j = len - 1; i <= j; i++, j--){
		char c = str[i];
		str[i] = str[j];
		str[j] = c;
	}
	return str;
}

/**
 * encodes crc values and appends them to send
 */
void encode_crc(){
	toStr();

	//appends n-1 zeros to data
	strcpy(appended_data, "000");
	strcat(appended_data, data_str);
	strcpy(appended_data, str_rev(appended_data));

	division();

	//appends data and remainder
	char temp[10];
	strcpy(send_data, str_rev(data_str));
	strncpy(temp, remain+1, 4);
	strcat(send_data, temp);

	char *endptr;
	TxData[0] = strtol(data_str, &endptr, 2);
	TxData[1] = strtol(send_data, &endptr, 2);
}

/**
 * sends data with crc every 10ms
 */
void sendData(){
	encode_crc();

	HAL_Delay(10);
	HAL_UART_Transmit(&huart1, (uint8_t*)TxData, sizeof(TxData), 1000);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  sendData();
	  	  if(bitmask_check(0)) //red
	  	  {
	  		  data = 1;
	  		  sendData();
	  		  data = 0;
	  		  bitmask_clear(0);
	  	  }
	  	  if(bitmask_check(1)) //green
	  	  {
	  		  data = 2;
	  		  sendData();
	  		  data = 0;
	  		  bitmask_clear(1);
	  	  }
	  	  if(bitmask_check(2)) //yellow
	  	  {
	  		  data = 3;
	  		  sendData();
	  		  data = 0;
	  		  bitmask_clear(2);
	  	  }
	  	  if(bitmask_check(3)) //blue
	  	  {
	  		  data = 4;
	  		  sendData();
	  		  data = 0;
	  		  bitmask_clear(3);
	  	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_TIM16;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */
		//f = 8Mhz / PSC = 50kHz
		//T = (1 / f) * period = 20ms
  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 159;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	  UNUSED(GPIO_Pin);

  if (GPIO_Pin == GPIO_PIN_2){
	  state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
	  button = BUTTON_1;	//red
  }
  if (GPIO_Pin == GPIO_PIN_3){
	  state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
	  button = BUTTON_2;	//green
  }
  if (GPIO_Pin == GPIO_PIN_14){
	  state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	  button = BUTTON_3;	//yellow
  }
  if (GPIO_Pin == GPIO_PIN_15){
	  state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	  button = BUTTON_4;	//blue
  }
  HAL_TIM_Base_Start_IT(&htim16);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	 UNUSED(htim);

	if(htim == &htim16){
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == state && button == BUTTON_1){
			bitmask_set(0);
		}
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == state && button == BUTTON_2){
			bitmask_set(1);
		}
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == state && button == BUTTON_3){
			bitmask_set(2);
		}
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == state && button == BUTTON_4){
			bitmask_set(3);
		}
	}
	HAL_TIM_Base_Stop_IT(&htim16);
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
