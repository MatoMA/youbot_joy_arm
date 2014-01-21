/*!
* \file youbot_joy_teleop.cpp
* \brief Allows for control of the KUKA youBot with a joystick.
*
* youbot_joy_teleop creates a ROS node that allows the control of a KUKA youBot with a joystick.
* This node listens to a /joy topic and sends messages to the /cmd_vel topic. Arm control is currently unimplemented.
*
* \author Russell Toris, WPI - rctoris@wpi.edu
* \date May 21, 2013
*/

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <youbot_joy_arm/youbot_joy_teleop.h>

#include <iostream>
#include <assert.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <math.h>
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "std_msgs/String.h"

ros::Time T;
bool receivedmsg = false;

double joint[5];
double lastJoint[5];                
double gripperr = 0;
double gripperl = 0;

double jointMax[] = {5.840139, 2.617989, -0.0157081, 3.42919, 5.641589};
double jointMin[] = {0.01006921, 0.01006921, -5.0264, 0.0221391, 0.11062};
double gripperMax = 0.0115;
double gripperMin = 0;

double jointDelta[5];
double gripperDelta = (gripperMax - gripperMin) * 0.02;        

double jointHome[] = {0.01007,0.01007,-0.15709,0.02214,0.1107};
double jointCamera[] = {3.0,0.5,-0.9,0.1,3.0};
double jointObject[] = {3.04171,0.63597,-1.017845,0.36284,2.876194};
double jointGrasp[] = {3.04171,2.04427,-1.5189129,2.5434289757,2.8761944};
double jointInitialize[] = {0.01007,.635971,-1.91989,1.04424,2.87619};

using namespace std;

void position_listener(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(msg->name[0] == "arm_joint_1")
    {
        joint[0] = msg->position[0];
        joint[1] = msg->position[1];
        joint[2] = msg->position[2];
        joint[3] = msg->position[3];
        joint[4] = msg->position[4];

        gripperl = msg->position[5];
        gripperr = msg->position[6];
    }
}

youbot_joy_teleop::youbot_joy_teleop()
{
  // create the ROS topics
  //cmd_vel = node.advertise < geometry_msgs::Twist > ("cmd_vel", 10);
  joy_sub = node.subscribe < sensor_msgs::Joy > ("joy", 10, &youbot_joy_teleop::joy_cback, this);
  armPositionsPublisher = node.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
  gripperPositionPublisher = node.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);
  armPositionSubscriber = node.subscribe("joint_states", 1000, position_listener);

  const int numberOfArmJoints = 5;
  const int numberOfGripperJoints = 2;

  armJointPositions.resize(numberOfArmJoints); 
  gripperJointPositions.resize(numberOfGripperJoints);

  ROS_INFO("youBot Joystick Teleop Started");
}

void youbot_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(!receivedmsg)
  {
    receivedmsg = true;
  }
  T = ros::Time::now();

  // create the twist message
  geometry_msgs::Twist twist;
  // left joystick controls the linear movement
  twist.linear.x = joy->axes.at(1);
  twist.linear.y = joy->axes.at(0);
  twist.linear.z = 0;
  // right joystick controls the angular movement
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = joy->axes.at(2);
  // send the twist command
  //cmd_vel.publish(twist);

  //Joint 0 Cross up/down
  if(joy->axes.at(7) > 0) {
      joint[0] += jointDelta[0];
      ROS_INFO("Joint0: %f", joint[0]);
  } else if (joy->axes.at(7) < 0){
      joint[0] -= jointDelta[0];
      ROS_INFO("Joint0: %f", joint[0]);
  }

  //Joint 1 Cross left/right
  if(joy->axes.at(6) > 0) {
      joint[1] += jointDelta[1];
      ROS_INFO("Joint1: %f", joint[1]);
  } else if (joy->axes.at(6) < 0){
      joint[1] -= jointDelta[1];
      ROS_INFO("Joint1: %f", joint[1]);
  }

  //Joint 2 Button Y/A
  if(joy->buttons.at(3)) {
      joint[2] += jointDelta[2];
      ROS_INFO("Joint2: %f", joint[2]);
  }
  if(joy->buttons.at(0)) {
      joint[2] -= jointDelta[2];
      ROS_INFO("Joint2: %f", joint[2]);
  }

  //Joint 3 Button Y/A
  if(joy->buttons.at(2)) {
      joint[3] += jointDelta[3];
      ROS_INFO("Joint3: %f", joint[3]);
  }
  if(joy->buttons.at(1)) {
      joint[3] -= jointDelta[3];
      ROS_INFO("Joint3: %f", joint[3]);
  }

  //Joint 4 LT/RT
  if(joy->axes.at(2) < 0) {
      joint[4] += jointDelta[4];
      ROS_INFO("Joint4: %f", joint[4]);
  } 
  if(joy->axes.at(5) < 0) {
      joint[4] -= jointDelta[4];
      ROS_INFO("Joint4: %f", joint[4]);
  } 

  //Gripper LB(open)/RB(close)
  if(joy->buttons.at(4)) {
      gripperl = gripperMax;
      gripperr = gripperMax;
      ROS_INFO("Gripper Open");
  }
  if(joy->buttons.at(5)) {
      gripperl = gripperMin;
      gripperr = gripperMin;
      ROS_INFO("Gripper Close");
  }

  //for(int i = 0; i < numberOfArmJoints; i++)
  for(int i = 0; i < 5; i++)
  {
      jointName.str("");
      jointName << "arm_joint_" << (i + 1);

      if(joint[i] < jointMin[i])
          joint[i] = jointMin[i];
      if(joint[i] > jointMax[i])
          joint[i] = jointMax[i];

      armJointPositions[i].joint_uri = jointName.str();
      armJointPositions[i].value = joint[i];

      armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
  }

  if(gripperl < gripperMin)
      gripperl = gripperMin;
  if(gripperr < gripperMin)
      gripperr = gripperMin;
  if(gripperl > gripperMax)
      gripperl = gripperMax;
  if(gripperr > gripperMax)
      gripperr = gripperMax;

  gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
  gripperJointPositions[0].value = gripperl;
  gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

  gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
  gripperJointPositions[1].value = gripperr;
  gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

  command.positions = armJointPositions;
  armPositionsPublisher.publish(command);

  command.positions = gripperJointPositions;
  gripperPositionPublisher.publish(command);

  //for(int i = 0; i <numberOfArmJoints; i++)
  for(int i = 0; i <5; i++)
  {
      lastJoint[i] = joint[i];
  }
}

void youbot_joy_teleop::joy_check()
{
    if( (receivedmsg) && ( (ros::Time::now().toSec() - T.toSec() ) > .15) )
    {
        geometry_msgs::Twist zero;
        //cmd_vel.publish(zero);
    }
}


int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "youbot_joy_arm");

  // initialize the joystick controller
  youbot_joy_teleop controller;

  std::fill_n(joint, 5, 0);
  gripperl = 0;
  gripperr = 0;

  ros::Rate rate(10); //Hz
  static const int numberOfArmJoints = 5;
  static const int numberOfGripperJoints = 2;

  for(int i = 0; i < numberOfArmJoints; i++)
  {
      jointDelta[i] = (jointMax[i] - jointMin[i]) * 0.02;
  }        

  ros::spinOnce();

  for(int i = 0; i < numberOfArmJoints; i++)
  {
      lastJoint[i] = joint[i];
  }

  // continue until a ctrl-c has occurred
  while(ros::ok())
  {
    ros::spinOnce();

    controller.joy_check();

    rate.sleep();
  }
  //ros::spin();
  return 0;
}
