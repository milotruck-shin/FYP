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

#define DEFAULT_POS 50
#define GEAR_RATIO 8

typedef enum{
	JUMP_STANCE,
	JUMP_TAKEOFF,
	JUMP_INAIR,
	LANDING
}jump_state_t;

typedef enum{
	JMP_RESET,
	JMP_SET
}JMPFlagStatus;

#define MOTEUS_POSITION_JMP_CMD { \
    .position = 0.0f,                 \
    .velocity = 3.5f,   \
    .feedforward_torque = 0.0f,       \
    .kp_scale = 1.0f,                 \
    .kd_scale = 1.0f,                 \
    .max_torque = 6.0f, \
    .stop_position = __builtin_nanf(""),    \
    .watchdog_timeout = __builtin_nanf(""), \
    .velocity_limit = __builtin_nanf(""),   \
    .accel_limit = 4.0f       \
}


#define MOTEUS_STARTUP_SEQ_CMD { \
	.position = 0.0f, 			 \
	.velocity = 3.5f,		\
	.feedforward_torque		\
	.kp_scale = 2.0f,		\
	.kd_scale = 0.03f,		\
	.max_torque = 6.0f,		\
	.stop_position = __builtin_nanf(""),    \
	.watchdog_timeout = __builtin_nanf(""), \
	.velocity_limit = __builtin_nanf(""),   \
	.accel_limit = 4.0f       \
}

#define AP_EVENT   (1 << 0)		//event flag for air pressure, will trigger once the UART receive INT is triggered
#define POLL_EVENT   (1 << 1)	//event flag for torque query
#define JMP_EVENT   (1 << 2)	//event flag for jump event
#define PNEU_EVENT   (1 << 3)	//event flag for pneumatic cylinder release


void motor_init(void);
void motor_stop(void);
void JMP_cmd(float target_rev);
float cosine_rule (float a,float L,float initial_pos); //returns pos in revs
float inv_cosine_rule (float pos_in_revs, float L,float initial_pos);	//returns distance between joint 1 and end effector

jump_state_t state_supervision(float rate_of_torque, float torque);


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
