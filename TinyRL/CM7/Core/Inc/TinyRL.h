/*
 * TinyRL.h
 *
 *  Created on: Jan 27, 2024
 *      Author: ousjl
 */

#ifndef INC_TINYRL_H_
#define INC_TINYRL_H_

// Funtion Like Macro


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

#define ROBOT_MOTOR_MODIFY_PWM	TIM1->CCR1 = ROBOT_MotorPWM ; \
								TIM1->CCR2 = ROBOT_MotorPWM

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


#endif /* INC_TINYRL_H_ */
