#include "ros/ros.h"
#include "ekf_acc.hpp"
#include "models_acc.hpp"
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

// Prints orientation of head with zyz Euler angles
void print_orientation(VectorXd& x_t) {
  Vector4d qvec;
  get_q(qvec, x_t);
  Quaterniond q_t(qvec(0), qvec(1), qvec(2), qvec(3));
  Vector3d euler = q_t.toRotationMatrix().eulerAngles(2, 1, 2);
  printf("head zyz euler angles: %lf %lf %lf\n", euler(0), euler(1), euler(2));
}

void handle_feedback(FeedbackMsg msg) {
  VectorXd u_t(3);
  vector<double> angles(num_modules);

  u_t(0) = msg.gyro[0].x;
  u_t(1) = msg.gyro[0].y;
  u_t(2) = msg.gyro[0].z;

  for (size_t i = 0; i < num_modules; i++) {
    Vector3d alpha(msg.accelerometer[i].x, msg.accelerometer[i].y, msg.accelerometer[i].z);
    set_alpha(z_t, alpha, i, num_modules);
    angles[i] = msg.position[i];
    if (i == 0) i += 0.26;
    if (joint_state.position.size() <= i) { 
      joint_state.position.push_back(angles[i]);
    } else {
      joint_state.position[i] = angles[i];
    }
  }

  if (first) {
    ekf.initialize(z_t);
    first = false;
  }

  ekf.predict(u_t, dt);
  ekf.correct(z_t, angles);  
 
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
  
  pose.header.frame_id = "world";
  pose.child_frame_id = "link0";
  pose.transform.rotation.w = q_head.w();
  pose.transform.rotation.x = q_head.x();
  pose.transform.rotation.y = q_head.y();
  pose.transform.rotation.z = q_head.z();
}

int main(int argc, char **argv) {
  z_t.setZero();

  size_t statelen = state_length(num_modules);
  size_t sensorlen = sensor_length(num_modules);

  ekf.Q = 0.001*MatrixXd::Identity(3, 3);

  ekf.R = MatrixXd::Identity(sensorlen, sensorlen);

  ekf.S_t = 0.001*MatrixXd::Identity(3, 3);

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
  
  /*
  // Initialize group using hebiros node
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>("hebiros/add_group_from_names");
  AddGroupFromNamesSrv add_group_srv;
  add_group_srv.request.group_name = "RUSNAKE";
  add_group_srv.request.names = {"RUSnake Module #114",
        "RUSnake Module #113",
        "RUSnake Module #112",
        "RUSnake Module #111",
        "RUSnake Module #110",
        "RUSnake Module #109",
        "RUSnake Module #101",
        "RUSnake Module #107",
        "RUSnake Module #106",
        "RUSnake Module #105",
        "RUSnake Module #104",
        "RUSnake Module #103"
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
  */

  joint_pub = n.advertise<sensor_msgs::JointState>("/reusnake/joint_state", 100);
  meas_pub = n.advertise<hebiros::FeedbackMsg>("/reusnake/measurement_model", 100);
  ros::Subscriber feedback_sub = n.subscribe("/hebiros/RUSNAKE/feedback", 100, handle_feedback);

  tf2_ros::TransformBroadcaster pose_br;
  ros::Rate r(50); 
  while (ros::ok()) {
    pose.header.stamp = ros::Time::now();
    joint_state.header.stamp = pose.header.stamp;
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
    }
    meas_pub.publish(measurement);
    
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
