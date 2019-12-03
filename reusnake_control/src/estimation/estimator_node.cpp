#include "ros/ros.h"
#include <snake_control_bridge/JointFeedback.h>
#include <snake_control_bridge/JointCommand.h>
#include "ekf.hpp"
#include "models.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

/* This file performs estimation by subscribing to commands and feedback
 * from snake_control_bridge. It updates its estimate every time a new
 * command is received
 */

using namespace Eigen;

static const size_t num_modules = 13;

static VectorXd z_t(7*num_modules);
static EKF ekf(1, 1, num_modules);
static double last_time; // time stamp of last feedback message
static double t; // time stamp of current feedback message
static ros::Publisher joint_pub;
static sensor_msgs::JointState joint_state;
static geometry_msgs::TransformStamped pose;

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
  pose.transform.rotation.w = q_t(0);
  pose.transform.rotation.x = q_t(1);
  pose.transform.rotation.y = q_t(2);
  pose.transform.rotation.z = q_t(3);

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

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "world";
  pose.child_frame_id = "link0";
  pose.transform.translation.x = 0;
  pose.transform.translation.y = 0;
  pose.transform.translation.z = 0;
  
  joint_pub = n.advertise<sensor_msgs::JointState>("/reusnake/joint_state", 1000);
  ros::Subscriber feedback_sub = n.subscribe("snake_feedback", 1000, handle_feedback);
  ros::Subscriber command_sub = n.subscribe("snake_command", 1000, handle_command);

  tf2_ros::TransformBroadcaster pose_br;

  while (ros::ok()) {
    pose_br.sendTransform(pose);
    joint_pub.publish(joint_state);
    ros::spinOnce();
  }

  return 0;
}
