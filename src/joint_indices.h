#ifndef JOINT_INDICES_H_
#define JOINT_INDICES_H_

// Joint indices are taken from gazebo, by
//  printing out hte index and names of each
//  joint when model->GetJoints() is called

#define NUM_POWERED_JOINTS 20
int powered_joint_indices[NUM_POWERED_JOINTS] = {2, 7, 7, 8, 10, 18, 19, 21, 11, 12, 13, 14, 16, 17, 22, 23, 24, 25, 27, 28};

#define NECK_YAW 2
#define NECK_PITCH 6

#define LEFT_SHOULDER_PITCH 7
#define LEFT_SHOULDER_ROLL 8
#define LEFT_ELBOW 10

#define RIGHT_SHOULDER_PITCH 18
#define RIGHT_SHOULDER_ROLL 19
#define RIGHT_ELBOW 21

#define LEFT_HIP_YAW 11
#define LEFT_HIP_ROLL 12
#define LEFT_HIP_PITCH 13
#define LEFT_KNEE 14
#define LEFT_ANKLE_PITCH 16
#define LEFT_ANKLE_ROLL 17

#define RIGHT_HIP_YAW 22
#define RIGHT_HIP_ROLL 23
#define RIGHT_HIP_PITCH 24
#define RIGHT_KNEE 25
#define RIGHT_ANKLE_PITCH 27
#define RIGHT_ANKLE_ROLL 28

#endif