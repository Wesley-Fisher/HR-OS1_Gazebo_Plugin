#ifndef HROS1_AX12_SERVO_H_
#define HROS1_AX12_SERVO_H_

// Work in degrees for angle position
// TODO: find what units the GetVelocity() function returns


class hros1_ax12_servo
{
private:
	float position;
	float speed;

	float goal_position;
	float goal_speed;

	// Values taken from https://www.ce.utwente.nl/aigaion/attachments/single/1015
	// Using equation from https://en.wikipedia.org/wiki/PID_controller, "Ideal vs standard form"
	static float scale_factor;
	static float Kp;
	static float Ki; // Units of s
	static float Kd; // Units of s
	static float beta; // Unknown use/units

	float last_updated_time; // Units of us

	float proportional_error;
	float derivitive_error;
	float integral_error;

	float torque;

	static float max_torque; // Units of Nm 

public:
	hros1_ax12_servo();
	~hros1_ax12_servo();

	void UpdatePosition(float pos, float vel);

	void SetGoalPosition(float goal, float goal_speed);

	float GetTorque();

	void PrintDetails(int id);

private:
	void UpdateErrors();
	long GetTime();
};




#endif