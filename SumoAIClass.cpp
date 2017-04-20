/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * \file SumoAIClass.h
 *
 * \brief the simulated AI agent. Students should modify this file
 *
 * \author Zongyi Yang, https://www.youtube.com/watch?v=ezp7sibEMmA
 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include "SumoAIClass.h"

void SumoAIClass::driverAI1(const unsigned int num_distance_sensors, const unsigned int num_line_sensors,
				 const float * distance_sensor_rad, const float * distance_sensor_angle, const float * distance_sensor_max_sense,
				 const float * line_sensor_rad, const float * line_sensor_angle,
				 const float * distance_sensor_readings, const bool *line_sensor_readings)
{
	// example agent that will push other robots out of the ring

    // things for students to add:
	// -use the back line sensor
	// -the loser of the previous match places robot 2nd, so he can encode using switches
	//  as an initial input the pos/orient of robot and pos/orient of opponent for a bettery start
    // -modify the sensor placement to match the one on the student's robot
	// -students should add a state machine with some hysteresis based on sensor inputs

	// if the line sensor in front of the robot is triggered.
	// this prevents robot from driving out of ring
	if (line_sensor_readings[0] == 1)
	{
		// we will reverse for 100 simulation steps
		revflag = 1;
		revcounter = 100;
	}

    // counting down the reverse here
	if (revflag && revcounter>0)
		revcounter--;
	else
		revflag = 0;
    
	// do the reverse operation if we are reversing
	if (revflag)
	{
		L = -1; R = -1;
	}
	// if the sensor on the right sees something, go to right by maxing L and throttling R
	else if (distance_sensor_readings[0]!=distance_sensor_max_sense[0])
	{
		R = distance_sensor_readings[0]/distance_sensor_max_sense[0]; L = 1;
	}
	// if the sensor on the left sees something, go to left by maxing R and throttling L
	else if (distance_sensor_readings[2]!=distance_sensor_max_sense[2])
	{
		R = 1; L = distance_sensor_readings[2]/distance_sensor_max_sense[2];
	}
	// if the sensor in the center sees something, but left and right do not see anything, go straight
	else if (distance_sensor_readings[1]!=distance_sensor_max_sense[1])
	{
		L = 1; R = 1;
	}
	// distances sensors see nothing, just go straight
	else
	{
		L = 1; R = -1;
	}

	return;
}

void SumoAIClass::driverAI2(const unsigned int num_distance_sensors, const unsigned int num_line_sensors,
				 const float * distance_sensor_rad, const float * distance_sensor_angle, const float * distance_sensor_max_sense,
				 const float * line_sensor_rad, const float * line_sensor_angle,
				 const float * distance_sensor_readings, const bool *line_sensor_readings)
{
	// example dumb agent that just spins in a circle
	L = 0.25; R = 0.5;

	return;
}

SumoAIClass::SumoAIClass()
{ 
	// initialize your memory variables here
	revcounter = 0;
	revflag = 0;
}