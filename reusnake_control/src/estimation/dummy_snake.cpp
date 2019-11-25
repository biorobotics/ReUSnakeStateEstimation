#include "ros/ros.h"
#include <snake_control_bridge/JointFeedback.h>
#include <snake_control_bridge/JointCommand.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

/* This file is meant to be used for testing. It simulates the values that
 * snake_control_bridge would publish if the snake were performing a simple
 * idealized behavior (currently simulates the snake rotation about the z-axis
 * of the head frame without any joint commands)
 */

static const double g = 9.8;
// works if you only use 2 modules (remember to change the value in
// estimator_node.cpp as well)
static const size_t num_modules = 13;
snake_control_bridge::JointFeedback fbk;
snake_control_bridge::JointCommand cmd;

int main(int argc, char **argv) {
  ros::init(argc, argv, "snake");
  ros::NodeHandle n;
  
  double start_time = ros::Time::now().toSec();
  
  ros::Publisher feedback_pub = n.advertise<snake_control_bridge::JointFeedback>("snake_feedback", 1000);
  ros::Publisher command_pub = n.advertise<snake_control_bridge::JointCommand>("snake_command", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    fbk.js.header.stamp = ros::Time::now();
    fbk.js.header.frame_id = "snake";
    fbk.js.name = {};

    for (size_t i = 0; i < num_modules; i++) {
      double t = fbk.js.header.stamp.toSec() - start_time;
      double cmd_pos = 0;//sin(t + i);
      double pos = 0;//sin(t - 0.1 + i);

      sensor_msgs::Imu imu;
      imu.angular_velocity.x = 0;
      imu.angular_velocity.y = 0;
      imu.angular_velocity.z = 1;

      imu.linear_acceleration.x = g*cos(t - i*M_PI/2 + M_PI);
      imu.linear_acceleration.y = g*sin(t - i*M_PI/2);
      imu.linear_acceleration.z = 0;

      if (fbk.js.position.size() < num_modules) {
        fbk.js.position.push_back(pos);
        fbk.imu_arr.push_back(imu);
        cmd.angles.push_back(0);
        cmd.velocities.push_back(0);
        cmd.torques.push_back(0);
      } else {
        fbk.js.position[i] = pos;
        fbk.imu_arr[i] = imu;
        cmd.angles[i] = cmd_pos;
        cmd.velocities[i] = 0;
        cmd.torques[i] = 0;
      }
    }
    feedback_pub.publish(fbk);
    command_pub.publish(cmd);
    loop_rate.sleep();
  }

  return 0;
}
