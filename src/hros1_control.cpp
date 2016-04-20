/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Dave Coleman
 *  \desc   Example ROS plugin for Gazebo
 */
 
// TODO: Set up a ros node
//   subscribe to messages
//   make a set of joints
//   set up an object for the joint controller
//   set up code to apply forces on joints

// Gazebo-ROS libraries
#include <gazebo_plugins/gazebo_ros_template.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>

// Standard Libraries
#include <ostream>
#include <vector>

// Includes for my files
 #include "joint_indices.h"
 #include "hros1_ax12_servo.h"
 #include "default_positions.h"

typedef boost::shared_ptr<gazebo::physics::Joint> JointPtr;

using namespace std;

const unsigned int servo_update_interval = 50;


void SetNewJointGoal(int joint_index, int servo_index, float goal_pos, float goal_speed);
void UpdateJoint(int joint_index, int servo_index);

namespace gazebo
{

class HROS1_Control_Plugin : public ModelPlugin
{

  physics::ModelPtr _model;
  physics::WorldPtr _world;

  std::vector<JointPtr> joints;
  hros1_ax12_servo servos[NUM_POWERED_JOINTS];

private:
  event::ConnectionPtr updateConnection;

// Constructor
public:
////////////////////////////////////////////////////////////////////////////////
HROS1_Control_Plugin() : ModelPlugin()
{
  cout << "Plugin Started" << endl;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
~HROS1_Control_Plugin()
{
}



////////////////////////////////////////////////////////////////////////////////
// Update the controller
void UpdateChild()
{
}

void OnUpdate(const common::UpdateInfo &)
{
  //cout << "Update" << endl;

  for (int i = 0; i < NUM_POWERED_JOINTS; i++)
  {
    int j = powered_joint_indices[i];

    UpdateJoint(j, i);
  }
}

void SetNewJointGoal(int joint_index, int servo_index, float goal_pos, float goal_speed)
{
  servos[servo_index].SetGoalPosition(goal_pos, goal_speed);
  UpdateJoint(joint_index, servo_index);
}

void UpdateJoint(int joint_index, int servo_index)
{
  float pos = joints[joint_index]->GetAngle(0).Degree();
  float vel = joints[joint_index]->GetVelocity(0);

  servos[servo_index].UpdatePosition(pos, vel);

  float t = servos[servo_index].GetTorque();

  joints[joint_index]->SetForce(0, t);

  servos[servo_index].PrintDetails(joint_index);
}




////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  _model = _parent;
  _world = _model->GetWorld();
  
  // Do some quick research into how to find the joints
  int num = _parent->GetJointCount();
  cout << "Number of joints: " << num << endl;
  
  this->joints = _parent->GetJoints();
  int size = joints.size();
  for (int i = 0; i < size; i++)
  {
    std::string s = joints[i]->GetName();
    cout << i << ": " << s << endl;
  }
  cout << endl << endl;

  // Actual Code

  // Initialize all joint angles
  for (int i = 0; i < NUM_POWERED_JOINTS; i++)
  {
    int j = powered_joint_indices[i];
    
    float goal = standing_positions[i];
    float goal_rad = goal * 3.1415 / 180.0;

    joints[j]->SetAngle(0, gazebo::math::Angle(goal_rad));

    SetNewJointGoal(j, i, goal, 0.0);
  }

  //StartJointUpdateTimer(TimedUpdateEvent, servo_update_interval);
  //boost::asio::io_service io;
  //boost::asio::deadline_time t(io, boost::posix_time::milliseconds(50));

  // Try to update controller on each sim iteration
  // http://gazebosim.org/tutorials/?tut=plugins_model
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&HROS1_Control_Plugin::OnUpdate, this, _1));
}


};



// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HROS1_Control_Plugin);
}


