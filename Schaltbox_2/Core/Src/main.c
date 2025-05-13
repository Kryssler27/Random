/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buff[30];
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(5000);
  HAL_TIM_Base_Start(&htim1);
  uint16_t CalSetup[5] ={0,0,0,0,0};
  #define RX_BUFF_SIZE 30
  #define CMD_DELAY_MS 10
  #define TOGGLE_ERROR() { \
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); \
		HAL_Delay(500); \
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); \
		HAL_Delay(500); \
	}

	void ErrorLoop(void) {
		while (1) {
			TOGGLE_ERROR();
		}
	}

	void SendCommand(const char *cmd, uint16_t len) {
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buff, RX_BUFF_SIZE);
		HAL_Delay(CMD_DELAY_MS);
		HAL_UART_Transmit(&huart1, (uint8_t*)cmd, len, 100);
		HAL_Delay(CMD_DELAY_MS);
	}

	// Initialization
	HAL_Delay(5000);
	HAL_TIM_Base_Start(&htim1);

	// Clear
	SendCommand("\r", 1);
	SendCommand("RGVN\r", 5);

	// Parse RGVN response
	if (strncmp((char *)&rx_buff[5], "15399", 5) == 0) {
		for (int i = 0; i < 5; i++) CalSetup[i] = 1000 + i * 500;
	} else if (strncmp((char *)&rx_buff[5], "15400", 5) == 0) {
		for (int i = 0; i < 5; i++) CalSetup[i] = 2000;
	} else {
		ErrorLoop();
	}

	// Check RLMDXR
	SendCommand("RLMDXR\r", 7);
	if (rx_buff[7] != 170) {
		ErrorLoop();
	}

	// Check RGM
	SendCommand("RGM\r", 4);
	if (rx_buff[8] & 0b00001) {
		ErrorLoop();
	}

	// Check RLVA
	SendCommand("RLVA\r", 5);
	if (!(rx_buff[5] == 32 && rx_buff[6] == 32 && rx_buff[7] == '0')) {
		ErrorLoop();
	}

	// GPIO safety checks
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)) HAL_Delay(4000), ErrorLoop();
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) HAL_Delay(5000), ErrorLoop();
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) HAL_Delay(6000), ErrorLoop();

	// Main loop variables
	int prevCal = 0;
	char tx_buff[20];

	while (1) {
		// Determine input pin
		int PinNr = 8, PinNumber = 0;
		for (int i = 0; i < 5; i++) {
			if (HAL_GPIO_ReadPin(GPIOA, PinNr << i)) {
				PinNumber = i + 1;
				break;
			}
		}

		// Update calibration
		if (prevCal != CalSetup[PinNumber]) {
			prevCal = CalSetup[PinNumber];
			snprintf(tx_buff, sizeof(tx_buff), "RLCT %d\r", prevCal);

			HAL_UART_Transmit(&huart1, "\r", 1, 100);
			HAL_Delay(100);
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buff, RX_BUFF_SIZE);
			HAL_Delay(CMD_DELAY_MS);
			HAL_UART_Transmit(&huart1, (uint8_t*)tx_buff, strlen(tx_buff), 100);
			HAL_Delay(CMD_DELAY_MS);
		}

		// Fire condition
		if (!HAL_GPIO_ReadPin(uC_LON_GPIO_Port, uC_LON_Pin)) {
			if (!HAL_GPIO_ReadPin(Fire_IN_1_GPIO_Port, Fire_IN_1_Pin)) {
				HAL_GPIO_WritePin(uC_MDMOD_GPIO_Port, uC_MDMOD_Pin, GPIO_PIN_SET);
				__HAL_TIM_SET_COUNTER(&htim1, 0);
				while (__HAL_TIM_GET_COUNTER(&htim1) < 100);
				HAL_GPIO_WritePin(uC_MDMOD_GPIO_Port , uC_MDMOD_Pin, GPIO_PIN_RESET);
				HAL_Delay(2000);
			}
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 50-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(ILOCK0_uC_GPIO_Port, ILOCK0_uC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, uC_Status_1_Pin|uC_Status_2_Pin|uC_Status_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7|uC_MDMOD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ILOCK0_uC_Pin */
  GPIO_InitStruct.Pin = ILOCK0_uC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ILOCK0_uC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uC_Status_1_Pin */
  GPIO_InitStruct.Pin = uC_Status_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(uC_Status_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : uC_Status_2_Pin uC_Status_3_Pin */
  GPIO_InitStruct.Pin = uC_Status_2_Pin|uC_Status_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Rotate_pos_1_Pin Rotate_pos_2_Pin Rotate_pos_3_Pin Rotate_pos_4_Pin */
  GPIO_InitStruct.Pin = Rotate_pos_1_Pin|Rotate_pos_2_Pin|Rotate_pos_3_Pin|Rotate_pos_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Fire_IN_2_Pin Fire_IN_1_Pin uC_LON_Pin uC_LACTIVE_Pin
                           uC_SYSOK_Pin uC_LOFF_Pin */
  GPIO_InitStruct.Pin = Fire_IN_2_Pin|Fire_IN_1_Pin|uC_LON_Pin|uC_LACTIVE_Pin
                          |uC_SYSOK_Pin|uC_LOFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : uC_MDMOD_Pin */
  GPIO_InitStruct.Pin = uC_MDMOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(uC_MDMOD_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buff, 30); //You need to toggle a breakpoint on this line!
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
