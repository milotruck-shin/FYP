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
ADC_HandleTypeDef hadc5;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

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
/* Definitions for Torque_polling */
osThreadId_t Torque_pollingHandle;
const osThreadAttr_t Torque_polling_attributes = {
  .name = "Torque_polling",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for JMP_cmd */
osThreadId_t JMP_cmdHandle;
const osThreadAttr_t JMP_cmd_attributes = {
  .name = "JMP_cmd",
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
uint8_t UART3_rxBuffer = {0};
FDCAN_TxHeaderTypeDef tx_header;
uint8_t tx_data[8] = {0xDE,0xAD,0xBE,0xEF,0x01,0x02,0x03,0x04};
FDCAN_FilterTypeDef filter;
FDCAN_RxHeaderTypeDef rx_header;
uint8_t rx_buffer [8];
volatile uint8_t can_received_flag;

float L = 0.2;
float y_target;
volatile float theta_motor;
volatile float torque;
uint32_t torque_array[10];
float rate_of_torque;
volatile jump_state_t state;
uint8_t JMP_status = false;

static moteus_motor_t* motor;
JMPFlagStatus JMP_FLAG = JMP_RESET;
moteus_position_cmd_t *sv_cmd = MOTEUS_POSITION_JMP_CMD;


uint8_t counter=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC5_Init(void);
void AP_task(void *argument);
void Pneu_task(void *argument);
void Polling_task(void *argument);
void JMP_task(void *argument);

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
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_ADC5_Init();
  /* USER CODE BEGIN 2 */
  //STARTUP SEQUENCE
  //config the FDCAN tx headers here, and the filters

  motor_init();

  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 16, 0);  // TDCO = DataTimeSeg1 + 1
  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
  HAL_UART_Receive_IT(&huart3,UART3_rxBuffer,1);

  const moteus_result_t* r = moteus_set_query(motor); //sends query to motor
  float initial_pos = r->position;
  r->rezero_state = initial_pos;
  float desired_pos = (DEFAULT_POS / 360) * GEAR_RATIO + initial_pos;
  float max_pos_bounds = (90 / 360) * GEAR_RATIO + initial_pos;
  float total_accum_angle = desired_pos;

  moteus_position_cmd_t cmd = MOTEUS_STARTUP_SEQ_CMD;
  cmd.position = desired_pos;
  moteus_begin_position(motor, &cmd);

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

  /* creation of Torque_polling */
  Torque_pollingHandle = osThreadNew(Polling_task, NULL, &Torque_polling_attributes);

  /* creation of JMP_cmd */
  JMP_cmdHandle = osThreadNew(JMP_task, NULL, &JMP_cmd_attributes);

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
  * @brief ADC5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC5_Init(void)
{

  /* USER CODE BEGIN ADC5_Init 0 */

  /* USER CODE END ADC5_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC5_Init 1 */

  /* USER CODE END ADC5_Init 1 */

  /** Common config
  */
  hadc5.Instance = ADC5;
  hadc5.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc5.Init.Resolution = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation = 0;
  hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait = DISABLE;
  hadc5.Init.ContinuousConvMode = DISABLE;
  hadc5.Init.NbrOfConversion = 1;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc5.Init.DMAContinuousRequests = DISABLE;
  hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc5.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC5_Init 2 */

  /* USER CODE END ADC5_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
	if (UART3_rxBuffer == JMP){

		osEventFlagsSet(leg_stateHandle, AP_EVENT);

		UART3_rxBuffer = 0;  //reset value
	}

	HAL_UART_Receive_IT(huart,UART3_rxBuffer,1); //re-arm
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	osEventFlagsSet(leg_stateHandle,POLL_EVENT);

}

void motor_init(void){

	//Initialise CAN peripheral
	moteus_can_init(&hfdcan1);

	//Create motor instance (ID 1)
	motor = moteus_init(&hfdcan1, 1);
	if (!motor){
		printf("Failed to initialise motor\n");
		return;
	}

	motor ->timeout_ms = 50;
}

void motor_stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim5);
    const moteus_result_t* r = moteus_set_stop(motor);
    if (r) {
        printf("Motor stopped.\n");
    }
}

float cosine_rule (float a,float L)
{
	float A_rad = acosf((2*L*L - a*a )/2*L*L);
	float pos_in_revs = A_rad / 2*M_PI;
	float desired_pos = (DEFAULT_POS / 2*M_PI) * GEAR_RATIO + initial_pos;

	return desired_pos;
}

void JMP_cmd(float target_rev)
{
    moteus_position_cmd_t cmd = MOTEUS_POSITION_JMP_CMD;
    cmd->position=target_rev;
    moteus_begin_position(motor, &cmd);

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

jump_state_t state_supervision(float rate_of_torque, float torque)
{
    osMutexAcquire(sv_cmd_mtxHandle, osWaitForever);
	if (rate_of_torque<0.01f && rate_of_torque>-0.01f && torque > 1.0f && torque < 2.0f)
	{
		state = JUMP_STANCE;
	}

	else if (rate_of_torque<0 && torque < 1.0f && state == JUMP_STANCE)
	{
		state = JUMP_TAKEOFF;
	}

	else if (rate_of_torque<0.01f && rate_of_torque>-0.01f && torque > -0.5f && torque < 0.5f && state == JUMP_TAKEOFF)
	{
		state = JUMP_INAIR;
	}

	else if (rate_of_torque > 3.0f && torque > 3.0f && state == JUMP_INAIR)
	{
		state = LANDING;
	}


    switch (state)
    {
		case JUMP_STANCE :
		{
			moteus_position_cmd_t *cmd = MOTEUS_POSITION_JMP_CMD;
			cmd->kp_scale = 2.0f;
			cmd->kd_scale = 0.3f;
			return state;
		}

		case JUMP_TAKEOFF :
		{
			moteus_position_cmd_t *cmd = MOTEUS_POSITION_JMP_CMD;
			cmd->kp_scale = 1.5f;
			cmd->kd_scale = 0.01f;
			osEventFlagsSet(leg_stateHandle, PNEU_EVENT);
			return state;
		}
		case JUMP_INAIR :
		{
			moteus_position_cmd_t *cmd = MOTEUS_POSITION_JMP_CMD;
			cmd->kp_scale = 1.5f;
			cmd->kd_scale = 0.01f;
			return state;
		}
		case LANDING :
		{
			moteus_position_cmd_t *cmd = MOTEUS_POSITION_JMP_CMD;
			cmd->kp_scale = 2.0f;
			cmd->kd_scale = 0.01f;
			return state;
		}

		default :
		{
			return state;
		}
    }
    osMutexRelease(sv_cmd_mtxHandle);

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
	osEventFlagsSet(leg_stateHandle, JMP_EVENT);
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

/* USER CODE BEGIN Header_Polling_task */
/**
* @brief Function implementing the Torque_polling thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Polling_task */
void Polling_task(void *argument)
{
  /* USER CODE BEGIN Polling_task */
  /* Infinite loop */
  for(;;)
  {
    osEventFlagsWait(leg_stateHandle, POLL_EVENT, osFlagsWaitAll,osWaitForever);

    if (!JMP_status)
    {
		osMutexAcquire(sv_cmd_mtxHandle, osWaitForever);
		moteus_begin_query(motor);
		osMutexRelease(sv_cmd_mtxHandle);


		if (counter == 10)
		{
			rate_of_torque=(torque_array[9]-torque_array[0]) / (10*0.1f);
			torque = torque_array[9];
			counter = 0;
			state_supervision(rate_of_torque,torque); //TO BE CHANGED
		}
		torque_array[counter++] = motor->result.torque;

    }
  }
  /* USER CODE END Polling_task */
}

/* USER CODE BEGIN Header_JMP_task */
/**
* @brief Function implementing the JMP_cmd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_JMP_task */
void JMP_task(void *argument)
{
  /* USER CODE BEGIN JMP_task */
  /* Infinite loop */
  for(;;)
  {
    osEventFlagsWait(leg_stateHandle,JMP_EVENT,osFlagsWaitAll,osWaitForever);

    //once the flag is set, send command to JUMP and set the flag to start polling continuously.
    moteus_begin_query(motor);
    const moteus_result_t* r = &motor->result;
	theta_motor = r->position * 2.0f * M_PI ;
	torque = r->torque;

	y_target = -(L+0.05f);
	float theta_joint_new = cosine_rule(y_target, L);
	float target_position_motor = theta_joint_new/(2.0f*M_PI);
	JMP_cmd(target_position_motor);

	HAL_TIM_Base_Start_IT(&htim5);

  }
  /* USER CODE END JMP_task */
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
