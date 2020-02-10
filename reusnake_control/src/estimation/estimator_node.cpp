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

static const size_t num_modules = 10;
static const int feedback_freq = 100;
static const double dt = 1.0/feedback_freq;

static VectorXd z_t(sensor_length(num_modules));
static EKF ekf(1, 1, num_modules);
static ros::Publisher joint_pub;
static ros::Publisher meas_pub;
static sensor_msgs::JointState joint_state;
static geometry_msgs::TransformStamped pose;
static bool first = true;

// Previous position command
static vector<double> prev_cmd(num_modules);

// Prints orientation of head with zyz Euler angles
void print_orientation(VectorXd& x_t) {
  Vector4d qvec;
  get_q(qvec, x_t);
  Quaterniond q_t(qvec(0), qvec(1), qvec(2), qvec(3));
  Vector3d euler = q_t.toRotationMatrix().eulerAngles(2, 1, 2);
  printf("head zyz euler angles: %lf %lf %lf\n", euler(0), euler(1), euler(2));
}

// Prints angular velocity of head
void print_angular_velocity(VectorXd& x_t) {
  Vector3d w_t;
  get_w(w_t, x_t);
  printf("angular velocity of head: %lf %lf %lf\n", w_t(0), w_t(1), w_t(2));
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
 
  vector<double> angles;
  for (size_t i = 0; i < num_modules; i++) { 
    double theta = get_theta(ekf.x_t, i);
    angles.push_back(theta);
    if (joint_state.position.size() <= i) { 
      joint_state.position.push_back(theta);
    } else {
      joint_state.position[i] = theta;
    }
  }
  Vector4d q_t;
  get_q(q_t, ekf.x_t);

  // Filter estimates module 1 orientation. Now calculate head orientation
  transformArray transforms = makeUnifiedSnake(angles);
  // Orientation of module 1 wrt head
  Matrix3d R = transforms[1].block(0, 0, 3, 3);
  Quaterniond q_m1_head(R); 
  // Orientation of module 1 wrt world
  Quaterniond q_m1_world(q_t(0), q_t(1), q_t(2), q_t(3));

  Quaterniond q_head = q_m1_world*q_m1_head.conjugate();
  
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "world";
  pose.child_frame_id = "link0";
  pose.transform.rotation.w = q_head.w();
  pose.transform.rotation.x = q_head.x();
  pose.transform.rotation.y = q_head.y();
  pose.transform.rotation.z = q_head.z();

  cout << q_head.w() << " " << q_head.vec() << "\n\n";
}

int main(int argc, char **argv) {
  z_t.setZero();

  size_t statelen = state_length(num_modules);
  size_t sensorlen = sensor_length(num_modules);
  ekf.Q = 0.01*MatrixXd::Identity(statelen, statelen);
  ekf.Q.block(0, 0, 3, 3) *= 100;
  ekf.R = MatrixXd::Identity(sensorlen, sensorlen);
  ekf.R.block(0, 0, num_modules, num_modules) /= 100;
  ekf.R.block(4*num_modules, 4*num_modules, 3*num_modules, 3*num_modules) /= 10;
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
  meas_pub = n.advertise<hebiros::FeedbackMsg>("/reusnake/measurement_model", 100);
  ros::Subscriber feedback_sub = n.subscribe("/hebiros/RUSNAKE/feedback", 100, handle_feedback);

  tf2_ros::TransformBroadcaster pose_br;
  ros::Rate r(50); // 10 hz
  while (ros::ok()) {
    pose_br.sendTransform(pose);
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
    
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
