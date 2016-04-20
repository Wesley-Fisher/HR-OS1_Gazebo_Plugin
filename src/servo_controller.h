#ifndef SERVO_CONTROLLER_BASE_H_
#define SERVO_CONTROLLER_BASE_H_

# TODO: figure our which ones include the joint reference
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>

using namespace std;

class Servo_Controller
{
	int _id;
	std::String _name;
	JointPtr _joint;

	bool _powered;

	float _pos;
	float _speed;

	float _goal_pos;
	float _goal_speed;

	float last_updated_time;

	


public:
	virtual void SetGoal(float goal_pos, float goal_speed);
	virtual void Update();
	virtual void PrintDetails();

	virtual void Poll(std::String key)
};

#endif