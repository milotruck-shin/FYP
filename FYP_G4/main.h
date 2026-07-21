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
#define GEAR_RATIO 1.5 // 40 teeth upper pulley. 60 teeth lower pulley

typedef enum{
	STANCE,
	TAKEOFF,
	INAIR,
	BRACE
}leg_state_t;

typedef enum{
	JMP_RESET,
	JMP_SET
}JMPFlagStatus;

#define MOTEUS_POSITION_JMP_CMD { \
		.position = 0.0f, 			 \
		.velocity = 0.0f,		\
		.feedforward_torque	= 0.0f,	\
		.kp_scale = 2.0f,		\
		.kd_scale = 0.03f,		\
		.max_torque = 0.3f,		\
		.stop_position = __builtin_nanf(""),    \
		.watchdog_timeout = __builtin_nanf(""), \
		.velocity_limit = 3.0f,   \
		.accel_limit = 1.5f       \
	}


#define MOTEUS_STARTUP_SEQ_CMD { \
	.position = 0.0f, 			 \
	.velocity = 0.0f,		\
	.feedforward_torque	= 0.0f,	\
	.kp_scale = 2.0f,		\
	.kd_scale = 0.03f,		\
	.max_torque = 2.0f,		\
	.stop_position = __builtin_nanf(""),    \
	.watchdog_timeout = __builtin_nanf(""), \
	.velocity_limit = 3.0f,   \
	.accel_limit = 1.5f       \
}

#define AP_EVENT   (1 << 0)		//event flag for air pressure, will trigger once the UART receive INT is triggered
#define POS_EVENT   (1 << 1)	//event flag for jump event
#define PNEU_EVENT   (1 << 2)	//event flag for pneumatic cylinder release

//moteus functions
void motor_init(void);
void motor_stop(void);
static inline void JMP_cmd(moteus_motor_t* motor,float target_position_motor)
{
    moteus_position_cmd_t cmd = MOTEUS_POSITION_JMP_CMD;
    cmd.position=target_position_motor;
    moteus_begin_position(motor, &cmd);
}


//state machine functions
leg_state_t state_supervision(float rate_of_torque, float torque);
static leg_state_t lookup_table(float rate_of_torque, float torque);


//math functions
static inline float cosine_rule (float a,float L,float initial_pos)
{
	float A_rad = acosf((2*L*L - a*a )/2*L*L);
	float pos_joint_revs = A_rad / 2*M_PI;
	float pos_in_revs = pos_joint_revs * GEAR_RATIO + initial_pos;
	return pos_in_revs;
}

static inline float inv_cosine_rule (float pos_in_revs, float L,float initial_pos)
{
	float pos_joint_revs = (pos_in_revs - initial_pos)/GEAR_RATIO;
	float A_rad = pos_joint_revs * 2 * M_PI;
	float a2 = 2*L*L - 2 * L * L *cosf(A_rad);
	float a = sqrtf(a2);
	return a;
}


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
