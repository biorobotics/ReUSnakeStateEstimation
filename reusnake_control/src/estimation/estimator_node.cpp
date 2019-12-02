#include "ros/ros.h"
#include <snake_control_bridge/JointFeedback.h>
#include <snake_control_bridge/JointCommand.h>
#include "ekf.hpp"
#include "models.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

/* This file performs estimation by subscribing to commands and feedback
 * from snake_control_bridge. It updates its estimate every time a new
 * command is received
 */

using namespace Eigen;

static const size_t num_modules = 2;

static VectorXd z_t(7*num_modules);
static EKF ekf(0.5, 0.05, num_modules);
static double last_time; // time stamp of last feedback message
static double t; // time stamp of current feedback message
static ros::Publisher state_pub;
static ros::Publisher joint_pub;
static geometry_msgs::Pose pose;
static sensor_msgs::JointState joint_state;

// Prints orientation of head with zyz Euler angles
void print_orientation(VectorXd& x_t) {
  Vector4d qvec;
  get_head(qvec, x_t, num_modules);
  Quaterniond q_t(qvec(0), qvec(1), qvec(2), qvec(3));
  Vector3d euler = q_t.toRotationMatrix().eulerAngles(2, 1, 2);
  printf("head zyz euler angles: %lf %lf %lf\n", euler(0), euler(1), euler(2));
}

// Prints angular velocity of vc
void print_angular_velocity(VectorXd& x_t) {
  Vector3d w_t;
  get_w(w_t, x_t);
  printf("angular velocity of vc: %lf %lf %lf\n", w_t(0), w_t(1), w_t(2));
}

void handle_feedback(const snake_control_bridge::JointFeedback::ConstPtr& msg) {
  // Update time interval
  if (last_time != 0) {
    last_time = t;
  }
  t = msg->js.header.stamp.toSec();

  for (size_t i = 0; i < num_modules; i++) {
    set_phi(z_t, i, msg->js.position[i]);
    Vector3d alpha(msg->imu_arr[i].linear_acceleration.x,
                   msg->imu_arr[i].linear_acceleration.y,
                   msg->imu_arr[i].linear_acceleration.z);
    set_alpha(z_t, alpha, i, num_modules);

    Vector3d gamma(msg->imu_arr[i].angular_velocity.x,
                   msg->imu_arr[i].angular_velocity.y,
                   msg->imu_arr[i].angular_velocity.z);
    set_gamma(z_t, gamma, i, num_modules);
  }
  
  // First time receiving feedback
  if (last_time == 0) {
    last_time = t - 0.1;
    ekf.initialize(z_t);
  }
}

void handle_command(const snake_control_bridge::JointCommand::ConstPtr& msg) {
  // Avoids scenario where we receive a command before receiving any feedback
  if (last_time == 0) {
    return;
  }

  VectorXd u_t(num_modules);
  for (size_t i = 0; i < num_modules; i++) {
    u_t(i) = msg->angles[i];
  }

  ekf.predict(u_t, t - last_time);
  ekf.correct(z_t);  
 
  Vector4d q_t;
  get_head(q_t, ekf.x_t, num_modules);
  pose.orientation.w = q_t(0);
  pose.orientation.x = q_t(1);
  pose.orientation.y = q_t(2);
  pose.orientation.z = q_t(3);

  for (size_t i = 0; i < num_modules; i++) {
    if (joint_state.position.size() <= i) {
      joint_state.position.push_back(get_theta(ekf.x_t, i));
    } else {
      joint_state.position[i] = get_theta(ekf.x_t, i);
    }
  }
  print_orientation(ekf.x_t);
}

int main(int argc, char **argv) {
  last_time = 0;
  t = 0;

  z_t.setZero();

  ros::init(argc, argv, "estimator");
  ros::NodeHandle n;
  
  state_pub = n.advertise<geometry_msgs::Pose>("/reusnake/pose", 1000);
  joint_pub = n.advertise<sensor_msgs::JointState>("/reusnake/joint_state", 1000);
  ros::Subscriber feedback_sub = n.subscribe("snake_feedback", 1000, handle_feedback);
  ros::Subscriber command_sub = n.subscribe("snake_command", 1000, handle_command);

  while (ros::ok()) {
    state_pub.publish(pose);
    joint_pub.publish(joint_state);
    ros::spinOnce();
  }

  return 0;
}
