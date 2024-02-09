/*
 * TinyRL.h
 *
 *  Created on: Jan 27, 2024
 *      Author: ousjl
 */

#ifndef INC_TINYRL_H_
#define INC_TINYRL_H_

#include "stdint.h"
#include "main.h"

/* Funtion Like Macros Begin */
#define ROBOT_MOTOR_CLOCKWISE		\
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET ); \
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET ); \
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET ); \
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET ) \


#define ROBOT_MOTOR_COUNTERCLOCKWISE	\
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET ); \
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET ) ; \
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET ); \
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET ) \

#define ROBOT_MOTOR_MODIFY_PWM	TIM1->CCR1 = Robot_MotorsPMW ; \
								TIM1->CCR2 = Robot_MotorsPMW
/* Funtion Like Macros End */

/* MACROS Begin */

#define RL_OBSERVATION_TIME_BEFORE_REPORTING_ms 200

// 									*** Hardware ****
#define RL_LeftEncoder 	TIM2
#define RL_RightEncoder	TIM5

#define RL_RIGHT_WHEEL_RADIUS_m 0.025
#define RL_LEFT_WHEEL_RADIUS_m  0.025
#define RL_ROBOT_SPACING_m		0.250

/* Macros End */

typedef struct
{
	uint32_t	TimeStamp;
	uint8_t 	Terminated;
	float 		Angle;
	float		RightWheelVelocity;
	float		LeftWheelVelocity;
}RobotObservation;

typedef enum
{
	ROBOT_WaitingForOrders,
	ROBOT_UpdatingOrders,
	ROBOT_GivingBackObservations
}ROBOT_States;

typedef enum
{
	RL_WaitingForOrders,
	RL_ExecutingTask,
	RL_SendingReport
}RL_ActionExecutionStates;
/* Function Prototypes Begin */

float RL_RobotGetRightWheelVelocity (void) ;
float RL_RobotGetLeftWheelVelocity (void) ;
float RL_RobotGetCurrentAngle (void) ;
float RL_RobotResetParcouredDistances (void) ;

/* Function Prototypes End */

#endif /* INC_TINYRL_H_ */
