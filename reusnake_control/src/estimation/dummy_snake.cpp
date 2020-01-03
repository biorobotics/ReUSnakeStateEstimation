#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include "hebiros/FeedbackMsg.h"
#include <geometry_msgs/Vector3.h>

/* This file is meant to be used for testing. It simulates the values that
 * snake_control_bridge would publish if the snake were performing a simple
 * idealized behavior (currently simulates the snake rotation about the z-axis
 * of the head frame without any joint commands)
 */

static const double g = 9.8;
static const size_t num_modules = 13;

// Simulate a snake with head spinning about its z-axis
int main(int argc, char **argv) {
  ros::init(argc, argv, "snake");
  ros::NodeHandle n;
  
  double start_time = ros::Time::now().toSec();
  
  ros::Publisher feedback_pub = n.advertise<hebiros::FeedbackMsg>("/hebiros/RUSNAKE/feedback", 1000);

  ros::Rate loop_rate(100);

  hebiros::FeedbackMsg feedback;
  while (ros::ok()) {
    for (size_t i = 0; i < num_modules; i++) {
      double t = ros::Time::now().toSec() - start_time - 10;
      double pos = 0;
      if (i == 0) {
        pos = M_PI/2 + 0.01*sin(50*t);
      }

      geometry_msgs::Vector3 gyro;
      geometry_msgs::Vector3 accelerometer;

      if (t > 0) { 
        gyro.x = cos(i*M_PI/2) + 0.03*sin(60*t + 0.8);
        gyro.y = sin(i*M_PI/2) + 0.03*sin(40*t + 0.4);
        gyro.z = 0 + 0.02*sin(35*t + 0.2);

        accelerometer.x = -g*sin(t)*sin(i*M_PI/2) + 0.005*sin(60*t + 0.7);
        accelerometer.y = g*sin(t)*cos(i*M_PI/2) + 0.007*sin(50*t + 0.3);
        accelerometer.z = g*cos(t) + 0.051054*i + 0.025527 + 0.004*sin(50*t + 0.4);
      } else {
        gyro.x = 0;
        gyro.y = 0;
        gyro.z = 0;
        accelerometer.x = 0;
        accelerometer.y = 0;
        accelerometer.z = g;
      }

      if (feedback.position.size() < num_modules) {
        feedback.position.push_back(pos);
        feedback.gyro.push_back(gyro);
        feedback.accelerometer.push_back(accelerometer);
      } else {
        feedback.position[i] = pos;
        feedback.gyro[i] = gyro;
        feedback.accelerometer[i] = accelerometer;
      }
    }
    feedback_pub.publish(feedback);
    loop_rate.sleep();
  }

  return 0;
}
