#include "hros1_servo.h"
#include <time.h>
#include <sys/time.h>

#include <ostream>
#include <iostream>
#include <iomanip>

using namespace std;

float hros1_ax12_servo::scale_factor = 3.1415 / 180.0 * 0.001;
float hros1_ax12_servo::Kp = 0.1;
float hros1_ax12_servo::Ki = 0.001;
float hros1_ax12_servo::Kd = 0.01;//50000;
float hros1_ax12_servo::beta = 0.2 * hros1_ax12_servo::scale_factor;

float hros1_ax12_servo::max_torque = 1.5;

hros1_ax12_servo::hros1_ax12_servo()
{
	last_updated_time = GetTime();

	position = 0;
	speed = 0;

	goal_position = 0;
	goal_speed = 0;

	proportional_error = 0;
	derivitive_error = 0;
	integral_error = 0;

	torque = 0;
}

hros1_ax12_servo::~hros1_ax12_servo()
{
	
}

void hros1_ax12_servo::UpdatePosition(float pos, float vel)
{
	position = pos;
	speed = vel;

	UpdateErrors();
}

void hros1_ax12_servo::SetGoalPosition(float goal, float goal_speed)
{
	goal_position = goal;
	goal_speed = goal;
}

void hros1_ax12_servo::UpdateErrors()
{
	// Get Time, calculate change in s, and update times
	long t= GetTime();
	long dt = t - last_updated_time;
	last_updated_time = t;

	float dt_s = (float) dt / 1000000.0;

	// Calculate errors
	this->proportional_error = goal_position - position;
	derivitive_error = goal_speed - speed;
	integral_error += proportional_error * dt_s;
}

long hros1_ax12_servo::GetTime()
{
	// http://stackoverflow.com/questions/3756323/getting-the-current-time-in-milliseconds
	struct timeval spec;
	gettimeofday(&spec, NULL);

	return (long) (spec.tv_usec);
}

float hros1_ax12_servo::GetTorque()
{
	UpdateErrors();

	torque = (Kp * proportional_error) +
		     (Ki * integral_error) +
		   	 (Kd * derivitive_error);

	if (torque > max_torque)
	{
		torque = max_torque;
	}
	else if (torque < -max_torque)
	{
		torque = -max_torque;
	}

	return torque;
}

void hros1_ax12_servo::PrintDetails(int id)
{
	if ( id != 14 )
	{
		return;
	}
  	cout << setw(10);
  	cout << id << ": "
         << "(" << position << ", " << speed << ")->"
         << "(" << goal_position << ", " << goal_speed << "); "
         << "PID=(" << proportional_error << ", " << integral_error << ", " << derivitive_error << ") "
         << "->(" << Kp * proportional_error << ", " << Ki * integral_error << ", " << Kd * derivitive_error << ") "
         << "Torque=" << torque
         << endl;
}

