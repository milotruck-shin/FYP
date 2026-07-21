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
#include "cmsis_os.h"

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
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

TIM_HandleTypeDef htim5;

/* Definitions for APMonitor */
osThreadId_t APMonitorHandle;
const osThreadAttr_t APMonitor_attributes = {
  .name = "APMonitor",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for Pneu_trigger */
osThreadId_t Pneu_triggerHandle;
const osThreadAttr_t Pneu_trigger_attributes = {
  .name = "Pneu_trigger",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};
/* Definitions for position_cmd */
osThreadId_t position_cmdHandle;
const osThreadAttr_t position_cmd_attributes = {
  .name = "position_cmd",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for sv_cmd_mtx */
osMutexId_t sv_cmd_mtxHandle;
const osMutexAttr_t sv_cmd_mtx_attributes = {
  .name = "sv_cmd_mtx",
  .attr_bits = osMutexRecursive,
};
/* Definitions for leg_state */
osEventFlagsId_t leg_stateHandle;
const osEventFlagsAttr_t leg_state_attributes = {
  .name = "leg_state"
};
/* USER CODE BEGIN PV */


//UART
#define RX_BUF_SIZE 8
char HLPUART1_rxBuffer [RX_BUF_SIZE];
uint8_t HUART1_rxBuffer;

uint8_t uart_rx_index = 0;
uint8_t rx_index = 0;

uint8_t rx_data_SERIAL;
uint8_t rx_data_ESP32;

//FLAGS
uint8_t command_ready = 0;
uint8_t uart1_ready = 0;
static volatile uint8_t stopped = 0;

//button state
static uint8_t prevState = false;


// Leg parameters
float L = 0.25;
float y_target;
float initial_pos;
static float total_accum_angle;
volatile float torque;
uint32_t torque_array[10];
float rate_of_torque;
static volatile leg_state_t currentState = STANCE ;


static moteus_motor_t* motor;
JMPFlagStatus JMP_FLAG = JMP_RESET;
moteus_position_cmd_t sv_cmd = MOTEUS_POSITION_JMP_CMD;


static const moteus_result_t* r;
static volatile float target_position_motor;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
void AP_task(void *argument);
void Pneu_task(void *argument);
void pos_cmd_task(void *argument);

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
  MX_TIM5_Init();
  MX_ADC2_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //STARTUP SEQUENCE
  //config the FDCAN tx headers here, and the filters
  HAL_Delay(5000);

  //turn on active LOW relay
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  // pull STBY LOW for FDCAN normal mode MCP2562FD
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

  HAL_UART_Receive_IT(&huart1,&HUART1_rxBuffer,1);
  HAL_UART_Receive_IT(&hlpuart1,(uint8_t*)HLPUART1_rxBuffer,1);

  moteus_position_cmd_t cmd = MOTEUS_STARTUP_SEQ_CMD;


  r = moteus_set_query(motor); //sends query to motor
  initial_pos = r->position;
  float desired_pos = (DEFAULT_POS / 360) * GEAR_RATIO + initial_pos;
  float max_pos_bounds = (120 / 360) * GEAR_RATIO + initial_pos;
  total_accum_angle = desired_pos;

  cmd.position = desired_pos;
  moteus_begin_position(motor, &cmd);
  HAL_TIM_Base_Start_IT(&htim5);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Create the recursive mutex(es) */
  /* creation of sv_cmd_mtx */
  sv_cmd_mtxHandle = osMutexNew(&sv_cmd_mtx_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of APMonitor */
  APMonitorHandle = osThreadNew(AP_task, NULL, &APMonitor_attributes);

  /* creation of Pneu_trigger */
  Pneu_triggerHandle = osThreadNew(Pneu_task, NULL, &Pneu_trigger_attributes);

  /* creation of position_cmd */
  position_cmdHandle = osThreadNew(pos_cmd_task, NULL, &position_cmd_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of leg_state */
  leg_stateHandle = osEventFlagsNew(&leg_state_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
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
  hfdcan1.Init.ExtFiltersNbr = 1;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 2499;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 679;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

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
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	// Check if the NEW_MESSAGE bit is set to 1 inside the RxFifo0ITs variable"
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!= 0)
	{
		moteus_fdcan_rx_callback(hfdcan);
	}
}

void HAL_UART_RxCpItCallback(UART_HandleTypeDef *huart)
{


	if (huart->Instance == USART1)
	{
//			char msg [10];
//			sprintf(msg, "%02X\r\n", rx_data_ESP32);
//			HAL_UART_Transmit(&hlpuart1,(uint8_t*)msg,strlen(msg),10);
		HUART1_rxBuffer = rx_data_ESP32;

		if (HUART1_rxBuffer == 0x01)  // if X button is pressed
		{
			if (prevState == false)
			{
			  prevState = true;
			  stopped = 1;
			  motor_stop();

			  HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Motor stopped\r\n", 15, 100);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			}

			else
			{
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
			  target_position_motor= r->position;
			  stopped = 0;
			  prevState = false;
			}

		}
		else if (HUART1_rxBuffer == 0x02) // if circle button is pressed
		{
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
			  target_position_motor= r->position;
			  osEventFlagsSet(leg_stateHandle, AP_EVENT);

		}
		HUART1_rxBuffer = 0;

		HAL_UART_Receive_IT(huart, (uint8_t *)&rx_data_ESP32, 1);

	}
}


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
    motor->timeout_ms = 100;
}

void motor_stop(void)
{
    const moteus_result_t* r = moteus_set_stop(motor);
    if (r) {
        printf("Motor stopped.\n");
    }
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


void apply_torque(float torque_nm)
{
    printf("Applying %.2f Nm torque...\n", torque_nm);

    const moteus_result_t* r = moteus_set_torque(motor, torque_nm);
    if (r) {
        printf("Torque applied. Measured: %.3f Nm\n", r->torque);
    } else {
        printf("Command failed: %s\n", moteus_error_str(motor->last_error));
    }
}

static leg_state_t lookup_table(float rate_of_torque, float torque)
{

	if (rate_of_torque<0.05f && torque > 0.5f && torque < 1.3f)
	{
		currentState = STANCE;
	}

	else if (rate_of_torque<0 && torque < 1.0f && currentState == STANCE)
	{
		currentState = TAKEOFF;
	}

	else if (rate_of_torque<0.01f && rate_of_torque>-0.01f && torque > -0.1f && torque < 0.1f && currentState == TAKEOFF)
	{
		currentState = INAIR;
	}

	else if (rate_of_torque > 3.0f && torque > 3.0f && currentState == INAIR)
	{
		currentState = BRACE;
	}


	else if (rate_of_torque < 0.05f && torque > 0.5f && torque < 1.3f && currentState == BRACE)
	{
	  currentState = STANCE;
	}

	return currentState;
}

leg_state_t state_supervision(float rate_of_torque, float torque)
{
	currentState = lookup_table(rate_of_torque, torque);

    switch (currentState)
    {
		case STANCE :
		{
			sv_cmd.kp_scale = 2.0f;
			sv_cmd.kd_scale = 0.3f;
			return currentState;
		}

		case TAKEOFF :
		{
			sv_cmd.kp_scale = 1.5f;
			sv_cmd.kd_scale = 0.01f;
			osEventFlagsSet(leg_stateHandle, PNEU_EVENT);
			return currentState;
		}
		case INAIR :
		{
			sv_cmd.kp_scale = 1.5f;
			sv_cmd.kd_scale = 0.01f;
			return currentState;
		}
		case BRACE :
		{
			sv_cmd.kp_scale = 2.0f;
			sv_cmd.kd_scale = 0.01f;
			return currentState;
		}

		default :
		{
			return currentState;
		}
    }

}




/* USER CODE END 4 */

/* USER CODE BEGIN Header_AP_task */
/**
  * @brief  Function implementing the APMonitor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_AP_task */
void AP_task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	osEventFlagsWait(leg_stateHandle, AP_EVENT, osFlagsWaitAll, osWaitForever);
	// run code to read ADC value, then do compensation calculation.


	//finally set the flag for query event
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Pneu_task */
/**
* @brief Function implementing the Pneu_trigger thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Pneu_task */
void Pneu_task(void *argument)
{
  /* USER CODE BEGIN Pneu_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Pneu_task */
}

/* USER CODE BEGIN Header_pos_cmd_task */
/**
* @brief Function implementing the position_cmd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pos_cmd_task */
void pos_cmd_task(void *argument)
{
  /* USER CODE BEGIN pos_cmd_task */
  /* Infinite loop */
  for(;;)
  {
	    osEventFlagsWait(leg_stateHandle,POS_EVENT,osFlagsWaitAll,osWaitForever);
	    static uint8_t counter = 0;
	    static float torque_array[10];
	    //once the flag is set, send command to JUMP and set the flag to start polling continuously.
	    moteus_begin_query(motor);
	    r = &motor->result;
		torque_array[counter++] = r->torque;
	    osMutexAcquire(sv_cmd_mtxHandle, osWaitForever);
		if (!stopped)
		{
			JMP_cmd(motor,target_position_motor);
		}
	    sMutexRelease(sv_cmd_mtxHandle);

		if (counter == 10)
		{
			float rate_of_torque=(torque_array[9]-torque_array[0]) / (10*0.01f);
			float torque = torque_array[9];
			counter = 0;
			state_supervision(rate_of_torque,torque); //TO BE CHANGED

		}


  }
  /* USER CODE END pos_cmd_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	osEventFlagsSet(leg_stateHandle,POS_EVENT);

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
