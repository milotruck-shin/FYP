/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "moteus.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define MOTEUS_STARTUP_SEQ_CMD { \
	.position = 0.0f, 			 \
	.velocity = 3.5f,		\
	.feedforward_torque	= 0.0f,	\
	.kp_scale = 2.0f,		\
	.kd_scale = 0.03f,		\
	.max_torque = 6.0f,		\
	.stop_position = __builtin_nanf(""),    \
	.watchdog_timeout = __builtin_nanf(""), \
	.velocity_limit = __builtin_nanf(""),   \
	.accel_limit = 4.0f       \
}

HAL_StatusTypeDef moteus_begin_position(moteus_motor_t* motor,
                                         const moteus_position_cmd_t* cmd);

void motor_init(void);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
