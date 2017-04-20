/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * \file SumoBotClass.cpp
 *
 * \brief the simulated sumo robot
 *
 * \author Zongyi Yang, https://www.youtube.com/watch?v=ezp7sibEMmA
 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include "SumoBotClass.h"
#include <math.h>
#ifndef NULL
#define NULL 0
#endif


float SumoBotClass::getX()
{
	return pos_x;
}

float SumoBotClass::getY()
{
	return pos_y;
}

float SumoBotClass::getRadius()
{
	return radius;
}

float SumoBotClass::getRotation()
{
	return angle;
}


// RGB color of robot
float SumoBotClass::getR()
{
	if (R >1)
		return 1;
	else
		return R;
}

float SumoBotClass::getG()
{
	if (G >1)
		return 1;
	else
		return G;
}

float SumoBotClass::getB()
{
	if (B >1)
		return 1;
	else
		return B;
}


SumoBotClass::distance_sensor * SumoBotClass::getDistanceSensors()
{
	return distance_sensor_array;
}

SumoBotClass::line_sensor * SumoBotClass::getLineSensors()
{
	return line_sensor_array;
}

float * SumoBotClass::getDistanceSensorReadings()
{
	return distance_sensor_readings;
}

bool * SumoBotClass::getLineSensorReadings()
{
	return line_sensor_readings;
}

unsigned int SumoBotClass::getNumDistanceSensors()
{
	return num_distance_sensors;
}

unsigned int SumoBotClass::getNumLineSensors()
{
	return num_line_sensors;
}

// send sensor data ot the AI agent, use the L and R commands from the agent to get the force vector
void SumoBotClass::drive()
{
	// load sensor data
	float * distance_sensor_rad = new float[num_distance_sensors];
	float * distance_sensor_angle = new float[num_distance_sensors];
	float * distance_sensor_max_sense = new float[num_distance_sensors];
	for (unsigned int i = 0; i < num_distance_sensors; i++)
	{
		distance_sensor_rad[i] =  distance_sensor_array[i].rad;
		distance_sensor_angle[i] =  distance_sensor_array[i].angle;
		distance_sensor_max_sense[i] =  distance_sensor_array[i].max_sense;
	}
	float * line_sensor_rad = new float[num_line_sensors];
	float * line_sensor_angle = new float[num_line_sensors];
	float * line_sensor_max_sense = new float[num_line_sensors];
	for (unsigned int i = 0; i < num_line_sensors; i++)
	{
		line_sensor_rad[i] =  line_sensor_array[i].rad;
		line_sensor_angle[i] =  line_sensor_array[i].angle;
	}

	// send sensor data to the AI agent
	// TODO: in the future avoid hardcoding each agent, AI agent should just have a single driverAI1
	//       but the content depends on what we pass in to SumoBotClass. There sould be a parent virtual
	//       AI object
	if (AI!=NULL)
	{
		if (AItype == 1)
		{
			AI->driverAI1((const unsigned int) num_distance_sensors, (const unsigned int) num_line_sensors,
					(const float *) distance_sensor_rad, (const float *) distance_sensor_angle, (const float *) distance_sensor_max_sense,
					(const float *) line_sensor_rad, (const float *) line_sensor_angle,
					(const float *) distance_sensor_readings, (const bool *)line_sensor_readings);
		}
		else if (AItype == 2)
		{
			AI->driverAI2((const unsigned int) num_distance_sensors, (const unsigned int) num_line_sensors,
					(const float *) distance_sensor_rad, (const float *) distance_sensor_angle, (const float *) distance_sensor_max_sense,
					(const float *) line_sensor_rad, (const float *) line_sensor_angle,
					(const float *) distance_sensor_readings, (const bool *)line_sensor_readings);
		}
	}

	// return the motor commands and compute from that the driving angle and displacement (force)
	float L = AI->L;
	float R = AI->R;
	if (L>1)
		L = 1;
	else if (L<-1)
		L = -1;

	if (R>1)
		R = 1;
	else if (L<-1)
		R = -1;
	force = (L + R)/2 * motor_speed;
	angle += (R-L)*motor_speed;

	delete distance_sensor_rad;
	delete distance_sensor_angle;
	delete distance_sensor_max_sense;
	delete line_sensor_rad;
	delete line_sensor_angle;
	return;
}

// move based on angular space
void SumoBotClass::move()
{
	pos_x+=force*(float)cos(angle*PI/180);
	pos_y+=force*(float)sin(angle*PI/180);
}

// move based on cartesian space
void SumoBotClass::push(float fx, float fy)
{
	pos_x += fx;
	pos_y += fy;
}

// todo: handleing what to do on shutdown should be inside drive and not in the main
void SumoBotClass::shutdown()
{
	isOn = 0;
}

bool SumoBotClass::getStatus()
{
	return isOn;
}

SumoBotClass::SumoBotClass()
{
	pos_x = 0;
	pos_y = 0;
	radius = 0;
	R = 0;
	G = 0;
	B = 0;
	angle = 0;
	motor_speed = 0;
	force = 0;
	num_distance_sensors = 0;
	distance_sensor_array = NULL;
	distance_sensor_readings = NULL;
	num_line_sensors = 0;
	line_sensor_array = NULL;
	line_sensor_readings = NULL;
	AI = NULL;
	AItype = 0;
	isOn = 0;
}

SumoBotClass::SumoBotClass(float input_x, float input_y, float input_radius, float input_r, float input_g, float input_b,
		float start_angle, float input_motor_speed,
		unsigned int input_num_distance_sensors, unsigned int input_num_line_sensors,
		float * distance_sensor_angles, float * line_sensor_angles,
		float * distance_sensor_rad, float * distance_sensor_max_sense, float * line_sensor_rad,
		unsigned int input_AI)
{
	pos_x = input_x;
	pos_y = input_y;
	radius = input_radius;
	R = input_r;
	G = input_g;
	B = input_b;
	angle = start_angle;
	motor_speed = 1;
	force = 0;
	num_distance_sensors = input_num_distance_sensors;
	distance_sensor_array = new distance_sensor[num_distance_sensors];
	distance_sensor_readings = new float[num_distance_sensors];
	for (unsigned int i = 0; i < num_distance_sensors; i++)
	{
		distance_sensor_array[i].angle = distance_sensor_angles[i];
		distance_sensor_array[i].rad = distance_sensor_rad[i];
		distance_sensor_array[i].max_sense = distance_sensor_max_sense[i];
		distance_sensor_readings[i] = 0;
	}
	num_line_sensors = input_num_line_sensors;
	line_sensor_array = new line_sensor[num_line_sensors];
	line_sensor_readings = new bool[num_line_sensors];
	for (unsigned int i = 0; i < num_line_sensors; i++)
	{
		line_sensor_array[i].angle = line_sensor_angles[i];
		line_sensor_array[i].rad = line_sensor_rad[i];
		line_sensor_readings[i] = 0;
	}
	AI = new SumoAIClass();
	AItype = input_AI;
	isOn = 1;
}

SumoBotClass::~SumoBotClass()
{
	if (distance_sensor_array!=NULL)
		delete distance_sensor_array;
	if (distance_sensor_readings!=NULL)
		delete distance_sensor_readings;
	if (line_sensor_array!=NULL)
		delete line_sensor_array;
	if (line_sensor_readings!=NULL)
		delete line_sensor_readings;
	delete AI;
}