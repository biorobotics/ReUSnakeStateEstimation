#include "ros/ros.h"
#include "ekf_vc.hpp"
#include "models_vc.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include "hebiros/FeedbackMsg.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include "SnakeKinematics.h"
#include <iostream>

/* This file performs estimation by subscribing to feedback
 * from the hebiros node
 */

using namespace Eigen;
using namespace hebiros;

static const size_t num_modules = 10;
static const int feedback_freq = 100;
static const double dt = 1.0/feedback_freq;

static VectorXd z_t(7*num_modules);
static EKF ekf(1, 1, num_modules);
static ros::Publisher joint_pub;
static ros::Publisher meas_pub;

// Publish the expected imu measurement if there were an imu on the head
static ros::Publisher head_imu_pub; 
static sensor_msgs::Imu head_imu;

static sensor_msgs::JointState joint_state;
static geometry_msgs::TransformStamped pose;
static geometry_msgs::TransformStamped virtual_chassis;
static bool first = true;

// Previous position command
static vector<double> prev_cmd(num_modules);

// Prints orientation of vc with zyz Euler angles
void print_orientation(VectorXd& x_t) {
  Vector4d qvec;
  get_q(qvec, x_t);
  Quaterniond q_t(qvec(0), qvec(1), qvec(2), qvec(3));
  Vector3d euler = q_t.toRotationMatrix().eulerAngles(2, 1, 2);
  printf("vc zyz euler angles: %lf %lf %lf\n", euler(0), euler(1), euler(2));
}

// Prints angular velocity of vc
void print_angular_velocity(VectorXd& x_t) {
  Vector3d w_t;
  get_w(w_t, x_t);
  printf("angular velocity of vc: %lf %lf %lf\n", w_t(0), w_t(1), w_t(2));
}

void handle_feedback(FeedbackMsg msg) {
  VectorXd u_t(num_modules);

  for (size_t i = 0; i < num_modules; i++) {
    double theta = msg.position[i];

    set_phi(z_t, i, theta);
    
    Vector3d alpha(msg.accelerometer[i].x, msg.accelerometer[i].y, msg.accelerometer[i].z);
    set_alpha(z_t, alpha, i, num_modules);

    Vector3d gamma(msg.gyro[i].x, msg.gyro[i].y, msg.gyro[i].z);
    set_gamma(z_t, gamma, i, num_modules);

    if (first) {
      u_t(i) = 0;
    } else {
      u_t(i) = (msg.position_command[i] - prev_cmd[i])/dt;
    }
    prev_cmd[i] = msg.position_command[i];
  }

  if (first) {
    ekf.initialize(z_t);
    first = false;
  }
  
  ekf.predict(u_t, dt);
  ekf.correct(z_t);  
 
  for (size_t i = 0; i < num_modules; i++) { 
    double theta = get_theta(ekf.x_t, i);
    if (joint_state.position.size() <= i) { 
      joint_state.position.push_back(theta);
    } else {
      joint_state.position[i] = theta;
    }
  }
  Vector4d q_t;
  get_q(q_t, ekf.x_t);

  Quaterniond q_vc_world(q_t(0), q_t(1), q_t(2), q_t(3));
  Matrix3d vc_R = ekf.prev_vc.block(0, 0, 3, 3);
  Quaterniond q_vc_head(vc_R);
  Quaterniond q_head = q_vc_world*q_vc_head.conjugate();

  pose.header.frame_id = "world";
  pose.child_frame_id = "link0";
  pose.transform.rotation.w = q_head.w();
  pose.transform.rotation.x = q_head.x();
  pose.transform.rotation.y = q_head.y();
  pose.transform.rotation.z = q_head.z();

  Quaterniond vc_q(vc_R);
  virtual_chassis.header.frame_id = "link0";
  virtual_chassis.child_frame_id = "vc";
  virtual_chassis.transform.translation.x = ekf.prev_vc(0, 3);
  virtual_chassis.transform.translation.y = ekf.prev_vc(1, 3);
  virtual_chassis.transform.translation.z = ekf.prev_vc(2, 3);
  virtual_chassis.transform.rotation.w = vc_q.w();
  virtual_chassis.transform.rotation.x = vc_q.x();
  virtual_chassis.transform.rotation.y = vc_q.y();
  virtual_chassis.transform.rotation.z = vc_q.z();

  // Calculate the expected imu measurement from the head
  Vector3d head_accel;
  Vector3d head_gyro;
  get_head_kinematics(head_accel, head_gyro, ekf.x_t, num_modules, dt);

  head_imu.linear_acceleration.x = head_accel(0);
  head_imu.linear_acceleration.y = head_accel(1);
  head_imu.linear_acceleration.z = head_accel(2);
  
  head_imu.angular_velocity.x = head_gyro(0);
  head_imu.angular_velocity.y = head_gyro(1);
  head_imu.angular_velocity.z = head_gyro(2);
}

int main(int argc, char **argv) {
  z_t.setZero();

  size_t statelen = state_length(num_modules);
  size_t sensorlen = sensor_length(num_modules);

  ekf.Q = MatrixXd::Identity(statelen, statelen);
  ekf.R = MatrixXd::Identity(sensorlen, sensorlen);
  ekf.R.block(0, 0, num_modules, num_modules) /= 100;
  ekf.R.block(4*num_modules, 4*num_modules, 3*num_modules, 3*num_modules) /= 10;
  ekf.S_t = 0.01*MatrixXd::Identity(statelen, statelen);

  ros::init(argc, argv, "estimator");
  ros::NodeHandle n;
  
  // Initialize pose message 
  pose.header.frame_id = "world";
  pose.child_frame_id = "link0";
  pose.transform.translation.x = 0;
  pose.transform.translation.y = 0;
  pose.transform.translation.z = 0;
  
  pose.transform.rotation.w = 1;
  pose.transform.rotation.x = 0;
  pose.transform.rotation.y = 0;
  pose.transform.rotation.z = 0;
  
  // Initialize group using hebiros node
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>("hebiros/add_group_from_names");
  AddGroupFromNamesSrv add_group_srv;
  add_group_srv.request.group_name = "RUSNAKE";
  add_group_srv.request.names = {"RUSnake Module #13",
        "RUSnake Module #12",
        "RUSnake Module #11",
        "RUSnake Module #10",
        "RUSnake Module #9",
        "RUSnake Module #8",
        "RUSnake Module #6",
        "RUSnake Module #3",
        "RUSnake Module #2",
        "RUSnake Module #1"
};
  add_group_srv.request.families = {"*"};
  // Block until group is created
  if (!add_group_client.call(add_group_srv)) {
    cout << "Lookup of RUSNAKE failed.\n";  
    exit(1);
  } 
  
  // Set feedback frequency
  ros::ServiceClient set_freq_client = n.serviceClient<SetFeedbackFrequencySrv>("hebiros/set_feedback_frequency");
  SetFeedbackFrequencySrv set_freq_srv;
  set_freq_srv.request.feedback_frequency = feedback_freq;
  set_freq_client.call(set_freq_srv);

  joint_pub = n.advertise<sensor_msgs::JointState>("/reusnake/joint_state", 100);
  meas_pub = n.advertise<sensor_msgs::JointState>("/reusnake/measurement_model", 100);
  head_imu_pub = n.advertise<sensor_msgs::Imu>("/reusnake/head_imu", 100);
  ros::Subscriber feedback_sub = n.subscribe("/hebiros/RUSNAKE/feedback", 100, handle_feedback);

  tf2_ros::TransformBroadcaster pose_br;
  ros::Rate r(50); // 10 hz
  while (ros::ok()) {
    pose.header.stamp = ros::Time::now();
    virtual_chassis.header.stamp = ros::Time::now();
    pose_br.sendTransform(pose);
    pose_br.sendTransform(virtual_chassis);
    joint_pub.publish(joint_state);

    // Publish predicted measurement
    hebiros::FeedbackMsg measurement;
    for (size_t i = 0; i < num_modules; i++) {
      Vector3d acc_vec;
      get_alpha(acc_vec, ekf.h_t, i, num_modules);
      geometry_msgs::Vector3 acc_vec_ros;
      acc_vec_ros.x = acc_vec(0);
      acc_vec_ros.y = acc_vec(1);
      acc_vec_ros.z = acc_vec(2);
      measurement.accelerometer.push_back(acc_vec_ros);
      
      Vector3d gyro_vec;
      get_gamma(gyro_vec, ekf.h_t, i, num_modules);
      geometry_msgs::Vector3 gyro_vec_ros;
      gyro_vec_ros.x = gyro_vec(0);
      gyro_vec_ros.y = gyro_vec(1);
      gyro_vec_ros.z = gyro_vec(2);
      measurement.gyro.push_back(gyro_vec_ros);
    }
    meas_pub.publish(measurement);

    head_imu.header.stamp = ros::Time::now();
    head_imu_pub.publish(head_imu);
    
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
