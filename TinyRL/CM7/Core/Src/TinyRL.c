/*
 * TinyRL.c
 *
 *  Created on: Feb 1, 2024
 *      Author: Flooki
 */


#include "TinyRL.h"



float RL_RobotGetRightWheelVelocity ( void )
{
	static uint32_t RL_RobotOldVelocityTicks = 0 ;
	uint32_t RL_RobotCurrentVelocityTicks = 0 ;
	RL_RobotCurrentVelocityTicks = ( RL_RightEncoder->CNT - RL_RobotOldVelocityTicks ) ;

	RL_RobotOldVelocityTicks = RL_RobotCurrentVelocityTicks ;
	return (RL_RobotCurrentVelocityTicks / RL_RIGHT_WHEEL_RADIUS_m ) ;
}


float RL_RobotGetLeftWheelVelocity ( void )
{
	static uint32_t RL_RobotOldVelocityTicks = 0 ;
	uint32_t RL_RobotCurrentVelocityTicks = 0 ;
	RL_RobotCurrentVelocityTicks = ( RL_LeftEncoder->CNT - RL_RobotOldVelocityTicks ) ;

	RL_RobotOldVelocityTicks = RL_RobotCurrentVelocityTicks ;
	return (RL_RobotCurrentVelocityTicks / RL_LEFT_WHEEL_RADIUS_m ) ;
}

float RL_RobotGetCurrentAngle ( void )
{
	static float RL_RobotCurrentAngle_rad = 0 ;
	float RL_RobotAngleChanging_rad = 0 ;
	RL_RobotAngleChanging_rad = ( ( RL_RobotGetRightWheelVelocity () - RL_RobotGetLeftWheelVelocity() ) / RL_ROBOT_SPACING_m ) ;
	RL_RobotCurrentAngle_rad += RL_RobotAngleChanging_rad ;
	return ( RL_RobotCurrentAngle_rad * 180 / M_PI ) ;
}

float RL_RobotResetParcouredDistances (void)
{
	RL_RightEncoder->CNT = RL_RightEncoder->ARR / 2 ;
	RL_LeftEncoder->CNT = RL_LeftEncoder->ARR / 2 ;
}





