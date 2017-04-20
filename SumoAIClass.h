/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * \file SumoAIClass.h
 *
 * \brief the simulated AI agent. Students should modify this file
 *
 * \author Zongyi Yang, https://www.youtube.com/watch?v=ezp7sibEMmA
 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#pragma once

#ifndef PI
#define PI 3.14159265
#endif

class SumoAIClass
{
private:
    // students should put "memory" variables here, they will be retained between frames
	unsigned int revcounter;
	bool revflag;
public:
    // the AI agent must update L and R after the driverAI is executed
	float L, R;

    // todo: right now we hardcode 2 AI slots. In the future, there should be a 
	//       virtual AI class that each agent inherets from. Each agent should only
	//       have one AI strategy, and the SumoBotClass will call driveAI from the
	//       virtual parent
	void driverAI1(const unsigned int num_distance_sensors, const unsigned int num_line_sensors,
				 const float * distance_sensor_rad, const float * distance_sensor_angle, const float * distance_sensor_max_sense,
				 const float * line_sensor_rad, const float * line_sensor_angle,
				 const float * distance_sensor_readings, const bool *line_sensor_readings);
	void driverAI2(const unsigned int num_distance_sensors, const unsigned int num_line_sensors,
				 const float * distance_sensor_rad, const float * distance_sensor_angle, const float * distance_sensor_max_sense,
				 const float * line_sensor_rad, const float * line_sensor_angle,
				 const float * distance_sensor_readings, const bool *line_sensor_readings);
	SumoAIClass();
};