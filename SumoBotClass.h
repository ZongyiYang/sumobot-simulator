/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * \file SumoBotClass.h
 *
 * \brief the simulated sumo robot
 *
 * \author Zongyi Yang, https://www.youtube.com/watch?v=ezp7sibEMmA
 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#pragma once
#include "SumoAIClass.h"

#ifndef PI
#define PI 3.14159265
#endif

// todo: we are assuming constant unit mass and unit time at the moment, which is why we can add force to displacement

class SumoBotClass
{
private:
	float pos_x;
	float pos_y;

	float radius;
	
	float R, G, B; // color of this robot

	float angle, force; // the direction and magnitude of force provided by motors

	float motor_speed; // a scaling factor for speed (so you can have one robot that is 2x faster for example)
	
	unsigned int num_distance_sensors;
	struct distance_sensor{
		float rad; // rad and angle are wrt to robot, and gives where we put the sensor
		float angle;
		float max_sense;
	};
	distance_sensor * distance_sensor_array;
	float * distance_sensor_readings;
	unsigned int num_line_sensors;
	struct line_sensor{
		float rad; // rad and angle are wrt to robot, and gives where we put the sensor
		float angle;
	};
	line_sensor * line_sensor_array;
	bool * line_sensor_readings;

	// this is the AI of the agent. Students should modify the SumoAIClass object
	SumoAIClass * AI;
	unsigned int AItype; // if there are mutliple AIs, this will select which to use
	// Todo: the SumoAIClass should be created outside this object and the pointer to it passed in as a paramter
    //       the current AI selection method inside the drive function is very hardcoded, passing it in as a
	//       parameter would get around that


	bool isOn;
public:
	float getX(); float getY(); float getRadius();
	float getR(); float getG(); float getB();
	float getRotation();
	void push(float fx, float fy);
	struct distance_sensor * getDistanceSensors();
	line_sensor * getLineSensors();
	float * getDistanceSensorReadings();
	bool * getLineSensorReadings();
	unsigned int getNumDistanceSensors();
	unsigned int getNumLineSensors();
	void drive();
	void move();
	void shutdown();
	bool getStatus();
	SumoBotClass();
	SumoBotClass(float input_x, float input_y, float input_r, float input_radius, float input_g, float input_b, 
		float start_angle, float input_motor_speed,
		unsigned int input_num_distance_sensors, unsigned int input_num_line_sensors,
		float * distance_sensor_angles, float * line_sensor_angles,
		float * distance_sensor_rad, float * distance_sensor_max_sense, float * line_sensor_rad,
		unsigned int input_AI);
	~SumoBotClass();
};