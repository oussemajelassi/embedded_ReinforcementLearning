/*
 * TinyRL.h
 *
 *  Created on: Jan 27, 2024
 *      Author: ousjl
 */

#ifndef INC_TINYRL_H_
#define INC_TINYRL_H_

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
