#include "ros/ros.h"
#include "kf.hpp"
#include "models.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include "hebiros/FeedbackMsg.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "SnakeKinematics.h"
#include "VirtualChassisKinematics.h"
#include <iostream>

/* This file performs estimation by subscribing to feedback
 * from the hebiros node
 */

using namespace Eigen;
using namespace hebiros;

static const size_t num_modules = 12;
static const int feedback_freq = 100;
static const double dt = 1.0/feedback_freq;

size_t statelen;
size_t sensorlen;

static VectorXd z_t;
static EKF ekf;
static ros::Publisher joint_pub;
//static ros::Publisher meas_pub;

// Publish the expected imu measurement if there were an imu on the head
/*
static ros::Publisher head_imu_pub; 
static sensor_msgs::Imu head_imu;
*/

static sensor_msgs::JointState joint_state;
static geometry_msgs::TransformStamped head_frame;
static geometry_msgs::TransformStamped body_frame;
static ros::Publisher pose_pub;
static geometry_msgs::PoseWithCovarianceStamped head_pose;

static bool first = true;

// Previous position command
static vector<double> prev_cmd(num_modules);

static short body_frame_module;

void handle_feedback(FeedbackMsg msg) {
  VectorXd u_t;
  
  if (body_frame_module < 0) {
    u_t = VectorXd::Zero(num_modules);
  }
  else {
    // If we're using a module as the body frame, provide that module's
    // gyro measurement in the control signal
    u_t = VectorXd::Zero(num_modules + 3);
  }

  for (size_t i = 0; i < num_modules; i++) {
    double theta = msg.position[i];
    if (i == 1) theta += 0.26;

    set_phi(z_t, i, theta);
    
    Vector3d alpha(msg.accelerometer[i].x, msg.accelerometer[i].y, msg.accelerometer[i].z);
    set_alpha(z_t, alpha, i, num_modules);

    Vector3d gamma(msg.gyro[i].x, msg.gyro[i].y, msg.gyro[i].z);
    set_gamma(z_t, gamma, i, num_modules);

    if (i == body_frame_module - 1) {
      u_t.tail(3) = gamma;
    }

    double command = msg.position_command[i];
    if (i == 1) command += 0.26;
    if (first) {
      u_t(i) = 0;
    } else {
      u_t(i) = (command - prev_cmd[i])/dt;
    }
    prev_cmd[i] = command;
  }

  if (first) {
    ekf.initialize(num_modules, body_frame_module, z_t);
    for (size_t row = 0; row < 6; row++) {
      for (size_t col = 0; col < 6; col++) {
        head_pose.pose.covariance[6*row + col] = 0;
      }
    }
    first = false;
  }

  ekf.predict(u_t, dt);
  ekf.correct(z_t);   
}

int main(int argc, char **argv) {
  statelen = state_length(num_modules);
  sensorlen = sensor_length(num_modules);

  z_t = VectorXd::Zero(sensorlen);

  int body_frame_module_int;
  string gait;
  ros::init(argc, argv, "estimator");
  ros::NodeHandle n("~");
  n.getParam("body_frame", body_frame_module_int);
  body_frame_module = (short)body_frame_module_int;
  n.getParam("gait", gait);

  ekf.Q = 0.00001*MatrixXd::Identity(statelen - 1, statelen - 1);
  ekf.Q.block<3, 3>(0, 0) *= 1000;
  ekf.Q.block<3, 3>(9, 9) *= 100000;
  ekf.Q.block<3, 3>(6, 6) *= 100;
  ekf.Q.block(12 + num_modules, 12 + num_modules, num_modules, num_modules) *= 100;

  ekf.R = MatrixXd::Identity(sensorlen, sensorlen);
  ekf.R.block(0, 0, num_modules, num_modules) /= 100;
  ekf.R.block(num_modules, num_modules, 3*num_modules, 3*num_modules) /= 20;
  ekf.R.block(4*num_modules, 4*num_modules, 3*num_modules, 3*num_modules) /= 2;

  ekf.S_t = 0.001*MatrixXd::Identity(statelen - 1, statelen - 1);

  // Adjust covariances for virtual chassis estimator
  if (body_frame_module < 0) {
    if (gait.compare("roll") == 0) {
      ekf.Q.block<3, 3>(3, 3) *= 100;
      ekf.Q.block<3, 3>(6, 6) *= 100;
    }
    ekf.Q.block(12 + num_modules, 12 + num_modules, num_modules, num_modules) *= 10;
    ekf.R.block(0, 0, num_modules, num_modules) /= 100;
  }

  // Initialize pose messages
  head_frame.header.frame_id = "body_frame";
  head_frame.child_frame_id = "link0";

  body_frame.header.frame_id = "world";
  body_frame.child_frame_id = "body_frame";

  body_frame.transform.translation.x = 0;
  body_frame.transform.translation.y = 0;
  body_frame.transform.translation.z = 0;

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

  joint_pub = n.advertise<sensor_msgs::JointState>("/reusnake/joint_state", 5);
  /*
  meas_pub = n.advertise<hebiros::FeedbackMsg>("/reusnake/measurement_model", 100);
  head_imu_pub = n.advertise<sensor_msgs::Imu>("/reusnake/head_imu", 100);
  */
  pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/reusnake/head_pose", 5);

  ros::Subscriber feedback_sub;
  feedback_sub = n.subscribe("/hebiros/RUSNAKE/feedback", 5, handle_feedback);

  tf2_ros::TransformBroadcaster pose_br;
  ros::Rate r(50); 
  while (ros::ok()) {
    if (!first) {
      vector<double> angles;
      for (size_t i = 0; i < num_modules; i++) { 
        double theta = get_theta(ekf.x_t, i);
        angles.push_back(theta);
        double theta_dot = get_theta_dot(ekf.x_t, i, num_modules);
        if (joint_state.position.size() <= i) { 
          joint_state.position.push_back(theta);
          joint_state.velocity.push_back(theta_dot);
        } else {
          joint_state.position[i] = theta;
          joint_state.velocity[i] = theta_dot;
        }
      }
      Vector4d q_t = get_q(ekf.x_t);

      // Filter estimates body frame orientation. Now calculate head orientation
      transformArray transforms = makeUnifiedSnake(angles);
      Matrix4d vc = getSnakeVirtualChassis(transforms);
      makeVirtualChassisConsistent(ekf.vc, vc);

      // Transformation of head frame wrt body frame
      Matrix4d T_head_body;
      
      if (body_frame_module < 0) {
        T_head_body = vc.inverse();
      } else {
        T_head_body = transforms[body_frame_module].inverse();
      }

      Matrix3d R_head_body = T_head_body.block<3, 3>(0, 0);
      Quaterniond q_head_body(R_head_body);

      // Orientation of body wrt world
      Quaterniond q_body_world(q_t(0), q_t(1), q_t(2), q_t(3));

      body_frame.transform.rotation.w = q_body_world.w();
      body_frame.transform.rotation.x = q_body_world.x();
      body_frame.transform.rotation.y = q_body_world.y();
      body_frame.transform.rotation.z = q_body_world.z();

      Vector3d p_t = get_p(ekf.x_t);

      body_frame.transform.translation.x = p_t(0);
      body_frame.transform.translation.y = p_t(1);
      body_frame.transform.translation.z = p_t(2);

      Vector3d t_head_world = q_body_world.toRotationMatrix()*T_head_body.block<3, 1>(0, 3);
      t_head_world += p_t;
      head_pose.pose.pose.position.x = t_head_world(0);
      head_pose.pose.pose.position.y = t_head_world(1);
      head_pose.pose.pose.position.z = t_head_world(2);

      head_frame.transform.rotation.w = q_head_body.w();
      head_frame.transform.rotation.x = q_head_body.x();
      head_frame.transform.rotation.y = q_head_body.y();
      head_frame.transform.rotation.z = q_head_body.z();
      head_frame.transform.translation.x = T_head_body(0, 3);
      head_frame.transform.translation.y = T_head_body(1, 3);
      head_frame.transform.translation.z = T_head_body(2, 3);

      // Marginalization via Schur complement
      MatrixXd A(ekf.S_t.block<6, 6>(0, 0));
      MatrixXd B(ekf.S_t.block(6, 6, statelen - 7, statelen - 7));
      MatrixXd C(ekf.S_t.block(6, 0, statelen - 7, 6));
      MatrixXd new_S = A - C.transpose()*B.llt().solve(C);

      Quaterniond q_head_world = q_body_world*q_head_body;

      head_pose.pose.pose.orientation.w = q_head_world.w();
      head_pose.pose.pose.orientation.x = q_head_world.x();
      head_pose.pose.pose.orientation.y = q_head_world.y();
      head_pose.pose.pose.orientation.z = q_head_world.z();

      for (size_t row = 0; row < 6; row++) {
        for (size_t col = 0; col < 6; col++) {
          head_pose.pose.covariance[6*row + col] = new_S(row, col);
        }
      }

      /*
      // Calculate the expected imu measurement from the head
      Vector3d head_accel;
      Vector3d head_gyro;
      get_head_kinematics(head_accel, head_gyro, ekf.x_t, num_modules, dt,
                          body_frame_module, ekf.vc);

      head_imu.linear_acceleration.x = head_accel(0);
      head_imu.linear_acceleration.y = head_accel(1);
      head_imu.linear_acceleration.z = head_accel(2);
      
      head_imu.angular_velocity.x = head_gyro(0);
      head_imu.angular_velocity.y = head_gyro(1);
      head_imu.angular_velocity.z = head_gyro(2);
      */

      head_pose.header.stamp = ros::Time::now();
      pose_pub.publish(head_pose);

      head_frame.header.stamp = ros::Time::now();
      pose_br.sendTransform(head_frame);
      body_frame.header.stamp = head_frame.header.stamp;
      pose_br.sendTransform(body_frame);
      joint_state.header.stamp = head_frame.header.stamp;
      joint_pub.publish(joint_state);

      /*
      // Publish predicted measurement
      hebiros::FeedbackMsg measurement;
      for (size_t i = 0; i < num_modules; i++) {
        Vector3d acc_vec = get_alpha(ekf.h_t, i, num_modules);

        geometry_msgs::Vector3 acc_vec_ros;
        acc_vec_ros.x = acc_vec(0);
        acc_vec_ros.y = acc_vec(1);
        acc_vec_ros.z = acc_vec(2);
        measurement.accelerometer.push_back(acc_vec_ros);
        
        Vector3d gyro_vec = get_gamma(ekf.h_t, i, num_modules);

        geometry_msgs::Vector3 gyro_vec_ros;
        gyro_vec_ros.x = gyro_vec(0);
        gyro_vec_ros.y = gyro_vec(1);
        gyro_vec_ros.z = gyro_vec(2);
        measurement.gyro.push_back(gyro_vec_ros);
      }
      meas_pub.publish(measurement);

      head_imu.header.stamp = ros::Time::now();
      head_imu_pub.publish(head_imu);
      */
    }
    
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
