#include "ros/ros.h"
#include "ekf.hpp"
#include "models.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include "hebiros/FeedbackMsg.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"
#include <sensor_msgs/JointState.h>
#include "SnakeKinematics.h"
#include <iostream>

/* This file performs estimation by subscribing to feedback
 * from the hebiros node
 */

using namespace Eigen;
using namespace hebiros;

static const size_t num_modules = 2;
static const int feedback_freq = 100;
static const double dt = 0.01; // 1/feedback_freq

//static VectorXd z_t(7*num_modules);
static VectorXd z_t(4*num_modules);
static EKF ekf(1, 1, num_modules);
static ros::Publisher joint_pub;
static sensor_msgs::JointState joint_state;
static geometry_msgs::TransformStamped pose;
static bool first = true;

double unspiral(double input, int index)
{
  if (index % 4 == 0 || index % 4 == 3)
    return -input;
  return input;
}

// Prints orientation of head with zyz Euler angles
void print_orientation(VectorXd& x_t) {
  Vector4d qvec;
  get_q(qvec, x_t);
  Quaterniond q_t(qvec(0), qvec(1), qvec(2), qvec(3));
  Vector3d euler = q_t.toRotationMatrix().eulerAngles(2, 1, 2);
  // printf("head zyz euler angles: %lf %lf %lf\n", euler(0), euler(1), euler(2));
}

// Prints angular velocity of head
void print_angular_velocity(VectorXd& x_t) {
  Vector3d w_t;
  get_w(w_t, x_t);
  printf("angular velocity of head: %lf %lf %lf\n", w_t(0), w_t(1), w_t(2));
}

void handle_feedback(FeedbackMsg msg) {

  VectorXd u_t(num_modules + 3);

  for (size_t i = 0; i < num_modules; i++) {
    double theta = unspiral(msg.position[i], i);

    set_phi(z_t, i, theta);
    
    Matrix3d rot = rotZ(-((double)(i + 1))*M_PI/2.0);

    Vector3d alpha(msg.accelerometer[i].x, msg.accelerometer[i].y, msg.accelerometer[i].z);
    alpha = rot*alpha;
    set_alpha(z_t, alpha, i, num_modules);

    /*
    Vector3d gamma(msg.gyro[i].x, msg.gyro[i].y, msg.gyro[i].z);
    gamma = rot*gamma;
    set_gamma(z_t, gamma, i, num_modules);
    */

    u_t(i) = theta;
  
    if (joint_state.position.size() <= i) { 
      joint_state.position.push_back(msg.position[i]);
    } else {
      joint_state.position[i] = msg.position[i];
    }
  }

  if (first) {
    ekf.initialize(z_t);
    first = false;
  }
  
  Matrix3d rot = rotZ(-M_PI/2);
  Vector3d gamma(msg.gyro[0].x, msg.gyro[0].y, msg.gyro[0].z);
  gamma = rot*gamma;
  u_t.block(num_modules, 0, 3, 1) = gamma;
  /*
  Vector3d gamma;
  get_gamma(gamma, z_t, 0, num_modules);
  u_t.block(num_modules, 0, 3, 1) = gamma;
  */

  ekf.predict(u_t, dt);
  ekf.correct(z_t);  
/*
  ekf.predict(u_t, dt);
  Vector4d q_t_p;
  get_q(q_t_p, ekf.x_t);
  ekf.correct(z_t);  
  Vector4d q_t_c;
  get_q(q_t_c, ekf.x_t);
  Quaterniond q_t_pq(q_t_p(0), q_t_p(1), q_t_p(2), q_t_p(2));
  Quaterniond q_t_cq(q_t_c(0), -q_t_c(1), -q_t_c(2), -q_t_c(2));
  Quaterniond diff = q_t_pq*q_t_cq;
  Vector4d diff_vec(diff.w(), diff.x(), diff.y(), diff.z());
  cout << diff_vec << "\n\n";
  */
 
  Vector4d q_t;
  get_q(q_t, ekf.x_t);
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "world";
  pose.child_frame_id = "link0";
  pose.transform.translation.x = 0;
  pose.transform.translation.y = 0;
  pose.transform.translation.z = 0;
  pose.transform.rotation.w = q_t(0);
  pose.transform.rotation.x = q_t(1);
  pose.transform.rotation.y = q_t(2);
  pose.transform.rotation.z = q_t(3);

  print_orientation(ekf.x_t);
}

int main(int argc, char **argv) {
  z_t.setZero();

  size_t statelen = state_length(num_modules);
  size_t sensorlen = sensor_length(num_modules);
  ekf.R = MatrixXd::Identity(statelen, statelen);
  ekf.Q = MatrixXd::Identity(sensorlen, sensorlen);
  ekf.S_t = 0.01*MatrixXd::Identity(statelen, statelen);

  ros::init(argc, argv, "estimator");
  ros::NodeHandle n;
  
  // Initialize pose message 
  pose.header.stamp = ros::Time::now();
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
        "RUSnake Module #7",
        "RUSnake Module #6",
        "RUSnake Module #5",
        "RUSnake Module #4",
        "RUSnake Module #3",
        "RUSnake Module #2",
        "RUSnake Module #1"};
  add_group_srv.request.families = {"*"};
  // Block until group is created
  while (!add_group_client.call(add_group_srv)) {} 
  
  // Set feedback frequency
  ros::ServiceClient set_freq_client = n.serviceClient<SetFeedbackFrequencySrv>("hebiros/set_feedback_frequency");
  SetFeedbackFrequencySrv set_freq_srv;
  set_freq_srv.request.feedback_frequency = feedback_freq;
  set_freq_client.call(set_freq_srv);

  joint_pub = n.advertise<sensor_msgs::JointState>("/reusnake/joint_state", 100);
  ros::Subscriber feedback_sub = n.subscribe("/hebiros/RUSNAKE/feedback", 100, handle_feedback);

  tf2_ros::TransformBroadcaster pose_br;
  ros::Rate r(50); // 10 hz
  while (ros::ok()) {
    pose_br.sendTransform(pose);
    joint_pub.publish(joint_state);
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
