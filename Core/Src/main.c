
#include "main.h"
#include <string.h>
#include <stdlib.h>

#define PARAM_LENGTH            8
#define CMD_VCTR_SIZE           4
// Minimum duty cycle for the servo motor (2.5% or 0°)
#define DUTY_CYCLE_LOWER_BOUND  2.5
// Maximum duty cycle for the servo motor (12.5% or 180°)
#define DUTY_CYCLE_UPPER_BOUND  12.5
// Minimum pulse width for the servo motor
#define MIN_PULSE_WIDTH         6.375
// Maximum pulse width for the servo motor
#define MAX_PULSE_WIDTH         31.875

TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

uint8_t rxChar[2];
uint8_t rxBuffer[16];
char cmdVctr[CMD_VCTR_SIZE][PARAM_LENGTH];
volatile uint8_t cmdVctrIdx = 0;
volatile uint8_t rxIdx = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
void ParseCommand(void);
void SelectDeMuxChannel(uint8_t);
float clamp(float, float, float);
float map(float, float, float, float, float);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();

  HAL_UART_Receive_IT(&huart2, rxChar, 1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // Set the initial angle of the servo motor to 0°
  uint8_t pulse = (uint8_t)((DUTY_CYCLE_LOWER_BOUND / 100.0) * 255);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);

  while (1);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5625 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DMUX_A_Pin|DMUX_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DMUX_C_GPIO_Port, DMUX_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DMUX_A_Pin DMUX_B_Pin */
  GPIO_InitStruct.Pin = DMUX_A_Pin|DMUX_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DMUX_C_Pin */
  GPIO_InitStruct.Pin = DMUX_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DMUX_C_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Enables the UART interrupt to receive text from the terminal.
 * @param huart UART handle
 * @retval None
 * @note The expected UART instance is USART2, which corresponds to the USB virtual COM port
 *       from the Nucleo board.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // Ignore any other UART instance rather than USART2
  if (huart->Instance != USART2)
    return;

  // Add the received character to the buffer. Ignore Enter and Backspace
  if (rxChar[0] != 13 && rxChar[0] != 8) {
    rxBuffer[rxIdx++] = rxChar[0];
    HAL_UART_Transmit(&huart2, rxChar, 1, 100);
  }

  // Handle backspace. Remove the last character from the buffer
  if (rxChar[0] == 8 && rxIdx > 0) {
    rxIdx--;
    HAL_UART_Transmit(&huart2, (uint8_t*)"\b \b", 3, 100);
  }

  // Handle Enter. Reset the buffer and handle the command
  if (rxChar[0] == 13) {
    rxIdx = 0;
    // Handle the command
    ParseCommand();
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);
    memset(rxChar, 0, sizeof(rxChar));
  }

  // Free the UART to receive the next character
  HAL_UART_Receive_IT(&huart2, rxChar, 1);
}

/**
 * @brief Handles the command received from the terminal.
 */
void ParseCommand(void) {
  // Parse the command to get its parameters
  char *input = (char*) rxBuffer;
  char *charPointer = strtok(input, " ");
  strcpy(cmdVctr[cmdVctrIdx++], charPointer);

  // Continue parsing the command until the end of the buffer
  do {
    charPointer = strtok(NULL, " ");
    if (charPointer != NULL && cmdVctrIdx < CMD_VCTR_SIZE) {
        strcpy(cmdVctr[cmdVctrIdx++], charPointer);
    }
  } while (charPointer != NULL && cmdVctrIdx < CMD_VCTR_SIZE);

  // Reset the command vector index
  cmdVctrIdx = 0;

  // Get the action part
  char cmd[PARAM_LENGTH];
  strcpy(cmd, cmdVctr[0]);

  if (strcmp(cmd, "LED") == 0) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  } else if (strcmp(cmd, "PWM") == 0) {
    // Get the ID of the actuator
    char id[PARAM_LENGTH];
    strcpy(id, cmdVctr[1]);

    // Get the type of signal
    char param[PARAM_LENGTH];
    strcpy(param, cmdVctr[2]);
    float dutyCycle;

    // Check if the input is a percentage or a duty cycle value
    if (strcmp(param, "-p") == 0) {
      dutyCycle = atof(cmdVctr[3]);
      dutyCycle = map(dutyCycle, 0, 100, DUTY_CYCLE_LOWER_BOUND, DUTY_CYCLE_UPPER_BOUND);
    } else {
      dutyCycle = atof(param);
      dutyCycle = clamp(dutyCycle, DUTY_CYCLE_LOWER_BOUND, DUTY_CYCLE_UPPER_BOUND);
    }

    // Select the DeMux output channel
    SelectDeMuxChannel((uint8_t) atoi(id));

    // Calculate the pulse width and set the duty cycle of the signal
    uint8_t pulse = (uint8_t)((dutyCycle / 100.0) * 255);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
  }

  memset(rxBuffer, 0, sizeof(rxBuffer));
}

/**
 * @brief Selects the output channel of the DeMux.
 * 
 * Sets the appropriate pin values to pass a signal through the desired channel, which can 
 * be 1 out 8 channels
 * 
 * @param channel Channel to be selected
 * @retval None
 * 
 * @note The function maps the channel number to zero-index representation internally
 */
void SelectDeMuxChannel(uint8_t channel) {
  // Decrease in 1 to map to zero-index representation
  channel--;

  // Calculate the state for each pin
  uint8_t A = (channel & 0x01); // bit 0
  uint8_t B = (channel & 0x02) >> 1; // bit 1
  uint8_t C = (channel & 0x04) >> 2; // bit 2

  // Set each pin state
  HAL_GPIO_WritePin(DMUX_A_GPIO_Port, DMUX_A_Pin, A ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DMUX_B_GPIO_Port, DMUX_B_Pin, B ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DMUX_C_GPIO_Port, DMUX_C_Pin, C ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Clamps a value between a lower and upper bound.
 * 
 * @param value Value to be clamped
 * @param lowerBound Lower bound
 * @param upperBound Upper bound
 * 
 * @retval Clamped value
 */
float clamp(float input, float lowerBound, float upperBound) {
  return input < lowerBound
    ? lowerBound
    : input > upperBound
      ? upperBound
      : input;
}

/**
 * @brief Maps a value from one range to another.
 * 
 * @param value Value to be mapped
 * @param fromLower Lower bound of the input range
 * @param fromUpper Upper bound of the input range
 * @param toLower Lower bound of the output range
 * @param toUpper Upper bound of the output range
 * 
 * @retval Mapped value
 * 
 * @ref https://docs.arduino.cc/language-reference/en/functions/math/map/
 */
float map(float value, float fromLower, float fromUpper, float toLower, float toUpper) {
  return (value - fromLower) * (toUpper - toLower) / (fromUpper - fromLower) + toLower;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  __disable_irq();
  while (1);
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
