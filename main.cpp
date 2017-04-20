/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * \file main.cpp
 *
 * \brief runs the sumo robot simulation
 *
 * \author Zongyi Yang, https://www.youtube.com/watch?v=ezp7sibEMmA
 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// tested on Windows 7/8
// needs glut. Follow instructions on here: http://www.videotutorialsrock.com/opengl_tutorial/get_opengl_setup_windows/text.php
// 

// TODO:
// - ubuntu support
// - make virtual parent of AI class so people can have multiple AIs without hardcoding
// - right now everything is in relative units, unit mass, unit time, and unit forces. Should scale to real world units
// - add a cone/beam sensor such as sonar, encoders, maybe slip detection?
// - add fake gaussian noise to sensors
// - make so that the line sensor triggers on seeing white instead of just being in the white ring, which will allow for random map shapes or simulating line-following robots
// - make sumo bots square instead of circular, since most are square in shape
// - encapsulate code into libraries so student only sees SumoAIClass to prevent confusion on what to do
// - move to better simulation software, maybe use ROS?

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <ctime> 
#include <GL/glut.h>
#include <process.h>
#include <windows.h> 
#include <string>

using namespace std;
#include "SumoBotClass.h"

#define PI 3.14159265

#define WORLD_DIM_R1 200 // radius of white ring
#define WORLD_DIM_R2 205 // radius of white ring plus thickess of that ring
#define WORLD_POS_X 500
#define WORLD_POS_Y 250

// TODO: this should be a vector
#define NUM_ROBOTS 2
SumoBotClass * robots [NUM_ROBOTS];

//Called when a key is pressed
void handleKeypress(unsigned char key, int x, int y) {
	switch (key) {
		case 27: //Escape key
			exit(0);
	}
}

//Initializes 3D rendering
void initRendering() {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
}

// main simulation loop
void update(int value) { 
	float x1, y1, x2, y2;

	// check to see of the robots are out of bound, otherwise 
	// do the drive function, which will supply the AI agent with sensor data
	// and return the force vector of the robot
	for (unsigned int i = 0; i < NUM_ROBOTS; i++)
	{
		x1 = robots[i]->getX();
		y1 = robots[i]->getY();
		if (x1*x1+y1*y1>WORLD_DIM_R2*WORLD_DIM_R2) // robot out of bound
			robots[i]->shutdown();
		else
			robots[i]->drive();
	}

	// compute pushing between robot
	float distance;
	float dx, dy, dr;
	float ratio;
	float fx, fy;
	for (unsigned int i = 0; i < NUM_ROBOTS; i++)
	{
			for (unsigned int j = i; j < NUM_ROBOTS; j++)
			{
				if (i!=j)
				{
					x1 = robots[i]->getX();
					y1 = robots[i]->getY();
					x2 = robots[j]->getX();
					y2 = robots[j]->getY();
					dx = x1-x2;
					dy = y1-y2;
					distance = robots[i]->getRadius()+robots[j]->getRadius();
					if (dx*dx+dy*dy<distance*distance) // if two robots is touching, push them apart
					{
						ratio = pow((dx*dx+dy*dy)/(distance*distance), 0.5f);
						fx = dx*ratio/20;
						fy = dy*ratio/20;
						robots[i]->push(fx, fy);
						robots[j]->push(-fx, -fy);
					}
				}
			}
	}

    // from the combined displacement from driving and pushing, compute the movement
	for (unsigned int i = 0; i < NUM_ROBOTS; i++)
	{
		if (robots[i]->getStatus())
			robots[i]->move();
	}

    // TODO: these should be encapsulated into the SumoBotClass and not be in main
    // now compute the activation of the sensors
	float angle;
	float radius;
	float x;
	float y;
	float mag_sqr;

	float x1a, y1a, x2a, y2a, min_distance;
	bool flag;
	float sgn;
	float determinant;
	float discriminant;
	float magdy;
	float intx, inty;
	
	float tempcos, tempsin;
	for (unsigned int i = 0; i < NUM_ROBOTS; i++)
	{
		if (robots[i]->getStatus()) // only do sensor computation if the robot is still on
		{
			// compute the line-sensors activations 
			// we just make the line sensor turn on if it's in the white ring
			for (unsigned int j = 0; j < robots[i]->getNumLineSensors(); j++)
			{
				// get the line sensor position wrt to world
				angle = (robots[i]->getLineSensors()[j]).angle + robots[i]->getRotation();
				radius = (robots[i]->getLineSensors()[j]).rad;
				x = robots[i]->getX() + radius*cos(angle*PI/180);
				y = robots[i]->getY() + radius*sin(angle*PI/180);
				mag_sqr = x*x + y*y;

				// we just make the line sensor turn on if it's in the white ring
				if (mag_sqr>=WORLD_DIM_R1*WORLD_DIM_R1 && mag_sqr <= WORLD_DIM_R2*WORLD_DIM_R2)
					robots[i]->getLineSensorReadings()[j] = 1;
				else
					robots[i]->getLineSensorReadings()[j] = 0;
			}
			
			// compute the distance-sensors activations 
			for (unsigned int j = 0; j < robots[i]->getNumDistanceSensors(); j++)
			{
				// get the vector of the "beam" the distance sensor uses
				tempcos = cos((robots[i]->getRotation() + robots[i]->getDistanceSensors()[j].angle) * PI/180);
				tempsin = sin((robots[i]->getRotation() + robots[i]->getDistanceSensors()[j].angle) * PI/180);
				x1a = robots[i]->getX() + robots[i]->getDistanceSensors()[j].rad * tempcos;
				y1a = robots[i]->getY() + robots[i]->getDistanceSensors()[j].rad * tempsin;
				x2a = x1a + robots[i]->getDistanceSensors()[j].max_sense * tempcos;
				y2a = y1a + robots[i]->getDistanceSensors()[j].max_sense * tempsin;
				min_distance = robots[i]->getDistanceSensors()[j].max_sense;

				// find the intersection of this beam with any robots
				for (unsigned int k = 0; k < NUM_ROBOTS; k++)
				{
					if (i!=k)
					{
						// we're using line-circle interceptions
						x1 = x1a - robots[k]->getX();
						y1 = y1a - robots[k]->getY();
						x2 = x2a - robots[k]->getX();
						y2 = y2a - robots[k]->getY();
						dx = x2 - x1;
						dy = y2 - y1;
						dr = pow(dx*dx + dy*dy, 0.5f);
						determinant = x1*y2-x2*y1;
						discriminant = robots[k]->getRadius()*robots[k]->getRadius()*dr*dr-determinant*determinant;
						if (discriminant >= 0)
						{
							sgn = 1;
							if (dy<0)
								sgn = -1;

							magdy = dy;
							if (magdy<0)
								magdy *= -1;

							intx = (determinant * dy + sgn * dx * pow(discriminant, 0.5f))/(dr*dr);
							inty = (-determinant * dx + magdy * pow(discriminant, 0.5f))/(dr*dr);
							flag = 0;
							if (x2>=x1)
							{
								if (intx>=x1 && intx<=x2)
									flag = 1;
							}
							else
							{
								if (intx<=x1 && intx>=x2)
									flag = 1;
							}
							if (flag)
							{
								distance = pow((intx-x1)*(intx-x1) + (inty-y1)*(inty-y1), 0.5f);
								if (distance < min_distance)
									min_distance = distance;
							}
							if (discriminant != 0)
							{
								intx = (determinant * dy - sgn * dx * pow(discriminant, 0.5f))/(dr*dr);
								inty = (-determinant * dx - magdy * pow(discriminant, 0.5f))/(dr*dr);
								flag = 0;
								if (x2>=x1)
								{
									if (intx>=x1 && intx<=x2)
										flag = 1;
								}
								else
								{
									if (intx<=x1 && intx>=x2)
										flag = 1;
								}
								if (flag)
								{
									distance = pow((intx-x1)*(intx-x1) + (inty-y1)*(inty-y1), 0.5f);
									if (distance < min_distance)
										min_distance = distance;
								}
							}
						}
					}
				}
				robots[i]->getDistanceSensorReadings()[j] = min_distance;
			}
		}
	}

	glutPostRedisplay(); //Tell GLUT that the display has changed
	glutTimerFunc(0, update, 0); // loop this function forever
}

void glEnable2D() // enables 2d drawing
{
	int vPort[4];

   glGetIntegerv(GL_VIEWPORT, vPort);

   glMatrixMode(GL_PROJECTION);
   glPushMatrix();
   glLoadIdentity();

   glOrtho(0, vPort[2], 0, vPort[3], -1, 1);
   glMatrixMode(GL_MODELVIEW);
   glPushMatrix();
   glLoadIdentity();
}

void drawCircle(float radius)
{
	float x,y;
	glBegin(GL_LINE_LOOP);
	for(int j = 0; j < 360; j++)
	{
		x = (float)radius * cos(j * PI/180.0f);
		y = (float)radius * sin(j * PI/180.0f);
		glVertex2f(x,y);
	}
	glEnd();
}

void drawFillCircle(float radius)
{
	float x,y;
	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(0,0);
	for(int j = 0; j <= 360; j++)
	{
		x = (float)radius * cos(j * PI/180.0f);
		y = (float)radius * sin(j * PI/180.0f);
		glVertex2f(x,y);
	}
	glEnd();
}

void drawScene() { // draws everything
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	glTranslatef(WORLD_POS_X, WORLD_POS_Y, 0);
	glColor3f(1,1,1);
	drawFillCircle(WORLD_DIM_R2);
	glColor3f(0,0,0);
	drawFillCircle(WORLD_DIM_R1);
	
	for (unsigned int i = 0; i < NUM_ROBOTS; i++)
	{
		glPushMatrix();
		glTranslatef(robots[i]->getX(),robots[i]->getY(), 0);
		if (robots[i]->getStatus())
			glColor3f(robots[i]->getR(),robots[i]->getG(),robots[i]->getB());
		else
			glColor3f(1,0,0);
		
		drawCircle(robots[i]->getRadius());
		glRotatef(robots[i]->getRotation()-90, 0, 0, 1);
		glBegin(GL_LINES);
		glVertex2f (0,0);
		glVertex2f (0,robots[i]->getRadius());
		glEnd();
		if (robots[i]->getStatus())
		{
			for (unsigned int j = 0; j < robots[i]->getNumLineSensors(); j++)
			{
				glPushMatrix();
				glRotatef(robots[i]->getLineSensors()[j].angle, 0, 0, 1);
				glTranslatef(0,robots[i]->getLineSensors()[j].rad, 0);
				if (robots[i]->getLineSensorReadings()[j])
					glColor3f(1,0,0);
				else
					glColor3f(0,1,0);
				drawFillCircle(2);
				glPopMatrix();
			}
			for (unsigned int j = 0; j < robots[i]->getNumDistanceSensors(); j++)
			{
				glPushMatrix();
				glRotatef(robots[i]->getDistanceSensors()[j].angle, 0, 0, 1);
				glTranslatef(0,robots[i]->getDistanceSensors()[j].rad, 0);
				if (robots[i]->getDistanceSensorReadings()[j]==robots[i]->getDistanceSensors()[j].max_sense)
					glColor3f(1,1,1);
				else
					glColor3f(1,0,0);
				glBegin(GL_LINES);
				glVertex2f (0,0);
				glVertex2f (0,robots[i]->getDistanceSensorReadings()[j]);
				glEnd();
				glPopMatrix();
			}
		}
		glPopMatrix();
	}
	
	glPopMatrix();


	glutSwapBuffers();
}

int main(int argc, char** argv)
{
	//Initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(1000, 500);
	
	//Create the window
	glutCreateWindow("");
	//initRendering();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable2D();
	//Set handler functions
	glutDisplayFunc(drawScene);
	glutKeyboardFunc(handleKeypress);
	glutTimerFunc(1, update, 0); //Add a timer
	

	// example robots

	// sensors
	// TODO: these should be vectors
	float distance_sensor_angles [3]; // direction sensors are pointing in degrees relative ot the robot's front
	distance_sensor_angles[0] = -30;
	distance_sensor_angles[1] = 0;
	distance_sensor_angles[2] = 30;
	float distance_sensor_rad [3]; // distance the sensor is from the center of the robot
	distance_sensor_rad[0] = 10;
	distance_sensor_rad[1] = 10;
	distance_sensor_rad[2] = 10;
	float distance_sensor_max_sense [3]; // max sensing distance of line sensor
	distance_sensor_max_sense[0] = 100;
	distance_sensor_max_sense[1] = 100;
	distance_sensor_max_sense[2] = 100;

	float line_sensor_angles [2]; // one line sensor in front (0 deg), one in the back (180 deg)
	line_sensor_angles[0] = 0;
	line_sensor_angles[1] = 180;
	float line_sensor_rad [2]; // distance the sensor is from the center of the robot
	line_sensor_rad[0] = 15;
	line_sensor_rad[1] = 15;

	// create the robots
	for (unsigned int i = 0; i < NUM_ROBOTS; i++)
	{
		robots[i] = new SumoBotClass(50*cos(((360/NUM_ROBOTS*i))*PI/180), 50*sin(((360/NUM_ROBOTS*i))*PI/180), 20, 1, 1, 1, 
			(360/NUM_ROBOTS * i), 1,
			3, 2,
			distance_sensor_angles, line_sensor_angles,
			distance_sensor_rad, distance_sensor_max_sense, line_sensor_rad,
			(i==NUM_ROBOTS-1)?1:2); // which AI to use
	}
	// run main program loop
	glutMainLoop();
	
	system("pause");
    return 0;
}
