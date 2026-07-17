/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#define RX_BUF_SIZE 64
char HLPUART1_rxBuffer[RX_BUF_SIZE];
volatile uint8_t HUART1_rxBuffer;
volatile uint8_t uart_rx_index = 0;
volatile uint8_t rx_index = 0;
volatile uint8_t command_ready = 0;
volatile uint8_t stopped = 0;
volatile uint8_t uart1_ready = 0;

uint8_t rx_data_SERIAL;
uint8_t rx_data_ESP;


float target_position;
static moteus_motor_t* motor;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
 ITM_SendChar(ch);
 return(ch);
}

int _write(int32_t file, uint8_t *ptr, int32_t len)
{
	for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
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
  MX_FDCAN1_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // turn on relay, PC12
  HAL_Delay(5000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
  // pull STBY LOW, PA9
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, (hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1), 0U) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  motor_init();



  moteus_position_cmd_t cmd = MOTEUS_STARTUP_SEQ_CMD;
  HAL_UART_Receive_IT(&hlpuart1,&rx_data_SERIAL,1);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_data_ESP, 1);

  cmd.position = 0.0f;  // or whatever your home/default position is

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (command_ready)
	  {
	      command_ready = 0;
	      char *endptr;

	      // Add this: if user sends "stop", clear fault
	      if (strncmp(HLPUART1_rxBuffer, "STOP", 4) == 0)
	      {
	          motor_stop();
	          stopped = 1;

	          HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Motor stopped\r\n", 15, 100);
	          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	      }
	      else
	      {
	          float position = strtof(HLPUART1_rxBuffer, &endptr);
	          if (endptr == HLPUART1_rxBuffer) {
	              HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Invalid parameters\r\n", 20, 100);
	          } else {
	              cmd.position = position;
	              stopped = 0;
	          }
	      }
	      memset(HLPUART1_rxBuffer, 0, RX_BUF_SIZE);
	  }


	  else if (uart1_ready)
	  {
		  uart1_ready = 0;
		  if (HUART1_rxBuffer == 0x01)
		  {
	          motor_stop();
	          stopped = 1;

	          HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Motor stopped\r\n", 15, 100);
	          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
		  }
		  else if (HUART1_rxBuffer == 0x02)
		  {
              cmd.position = 0.0;
              stopped = 0;
		  }
		  HUART1_rxBuffer = 0;


	  }

	  else if (!stopped) {
	      moteus_begin_position(motor, &cmd);
	  }


	  HAL_Delay(10);
	  print_motor_state();


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 57;
  hfdcan1.Init.NominalTimeSeg1 = 112;
  hfdcan1.Init.NominalTimeSeg2 = 57;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 11;
  hfdcan1.Init.DataTimeSeg1 = 22;
  hfdcan1.Init.DataTimeSeg2 = 11;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {

        if (rx_data_SERIAL == '\r' || rx_data_SERIAL == '\n')
        {
        	char crlf[] = "\r\n";
        	HAL_UART_Transmit(huart,(uint8_t*)crlf,2,10);
            if (rx_index > 0)
            {
            	HLPUART1_rxBuffer[rx_index] = '\0';
                command_ready = 1;
            }

            rx_index = 0;
        }
        else if (rx_index < (RX_BUF_SIZE - 1))
        {
            HAL_UART_Transmit(huart,(uint8_t*)&rx_data_SERIAL,1,10);
            HLPUART1_rxBuffer[rx_index++] = rx_data_SERIAL;
        }
        else
        {
            // Buffer overflow protection
            rx_index = 0;
        }

        HAL_UART_Receive_IT(huart, (uint8_t *)&rx_data_SERIAL, 1);

    }

	else if (huart->Instance == USART1)
		{
//			char msg [10];
//			sprintf(msg, "%02X\r\n", rx_data_ESP);
//			HAL_UART_Transmit(&hlpuart1,(uint8_t*)msg,strlen(msg),10);
			HUART1_rxBuffer = rx_data_ESP;
			uart1_ready = 1;
			HAL_UART_Receive_IT(huart, (uint8_t *)&rx_data_ESP, 1);

		}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    moteus_fdcan_rx_callback(hfdcan);

}

void print_motor_state(void)
{
    const moteus_result_t* r = moteus_set_query(motor);

    if (r) {
        printf("%s, %.3f rev, %.3f rev/s, %.3f Nm, %.1f V, %.1f C, %.1f A, %.1f A\r\n", \
         		moteus_mode_str(r->mode), r->position, r->velocity ,r->torque,r->voltage,r->temperature, r->q_current, r->d_current );
//        printf("Mode: %s\r\n", moteus_mode_str(r->mode));
//        printf("Position: %.3f rev\r\n", r->position);
//        printf("Velocity: %.3f rev/s\r\n", r->velocity);
//        printf("Torque: %.3f Nm\r\n", r->torque);
//        printf("Voltage: %.1f V\r\n", r->voltage);
//        printf("Temperature: %.1f C\r\n", r->temperature);

        if (r->fault != MOTEUS_FAULT_NONE) {
            printf("FAULT: %s\r\n", moteus_fault_str(r->fault));
        }
    } else {
        printf("Query failed: %s\r\n", moteus_error_str(motor->last_error));
    }
}

//void print_motor_state(void)
//{
//    char buf[64];
//    int len;
//    const moteus_result_t* r = moteus_set_query(motor);
//
//    if (r) {
//        len = snprintf(buf, sizeof(buf), "Mode: %s\r\n", moteus_mode_str(r->mode));
//        HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
//
//        len = snprintf(buf, sizeof(buf), "Position: %.3f rev\r\n", r->position);
//        HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
//
//        len = snprintf(buf, sizeof(buf), "Velocity: %.3f rev/s\r\n", r->velocity);
//        HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
//
//        len = snprintf(buf, sizeof(buf), "Torque: %.3f Nm\r\n", r->torque);
//        HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
//
//        len = snprintf(buf, sizeof(buf), "Voltage: %.1f V\r\n", r->voltage);
//        HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
//
//        len = snprintf(buf, sizeof(buf), "Temperature: %.1f C\r\n", r->temperature);
//        HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
//
//        if (r->fault != MOTEUS_FAULT_NONE) {
//            len = snprintf(buf, sizeof(buf), "FAULT: %s\r\n", moteus_fault_str(r->fault));
//            HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
//        }
//    } else {
//        len = snprintf(buf, sizeof(buf), "Query failed: %s\r\n", moteus_error_str(motor->last_error));
//        HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
//    }
//}

void motor_init(void)
{
    // Initialize CAN peripheral
    moteus_can_init(&hfdcan1);

    // Create motor instance (ID 1)
    motor = moteus_init(&hfdcan1, 1);
    if (!motor) {
    	char failed_msg[]="Failed to initialize motor\r\n";
    	HAL_UART_Transmit(&hlpuart1,(uint8_t*)failed_msg,sizeof(failed_msg),10);
        return;
    }

    else
    {
    	char sucess_msg[]="Motor Initialised!\r\n";
    	HAL_UART_Transmit(&hlpuart1,(uint8_t*)sucess_msg,sizeof(sucess_msg),10);
    }

    // Optional: reduce timeout for faster failure detection
    motor->timeout_ms = 50;
}

void motor_stop(void)
{
    const moteus_result_t* r = moteus_set_stop(motor);
    if (r) {
        printf("Motor stopped.\n");
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
