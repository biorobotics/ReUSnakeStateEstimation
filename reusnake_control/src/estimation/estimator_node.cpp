#include "ros/ros.h"
#include "kf.hpp"
#include "models.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "SnakeKinematics.h"
#include "VirtualChassisKinematics.h"
#include <iostream>
#include <hebiros/FeedbackMsg.h>

using namespace Eigen;

static const size_t num_modules = 12;
static const double dt = 1.0/50;

size_t statelen;
size_t sensorlen;

static VectorXd z_t;
static VectorXd prev_joints(num_modules);
static double prev_joint_time;
static VectorXd u_t;
static EKF ekf;
static ros::Publisher joint_pub;

static sensor_msgs::JointState joint_state;
static geometry_msgs::TransformStamped head_frame;
static geometry_msgs::TransformStamped body_frame;
static ros::Publisher pose_pub;
static geometry_msgs::PoseWithCovarianceStamped head_pose;

static short body_frame_module;

static vector<bool> imus_ready(num_modules);
static bool joints_ready;

class handle_imu {
  private:
    size_t i;
  public:
    handle_imu(size_t _i) : i(_i) {}

    void operator() (const sensor_msgs::ImuConstPtr& msg) const {
      Matrix<double, 3, 1> accel;
      Matrix<double, 3, 1> gyro;
      accel(0) = msg->linear_acceleration.x;
      accel(1) = msg->linear_acceleration.y;
      accel(2) = msg->linear_acceleration.z;
      gyro(0) = msg->angular_velocity.x;
      gyro(1) = msg->angular_velocity.y;
      gyro(2) = msg->angular_velocity.z;

      set_alpha(z_t, accel, i);
      set_gamma(z_t, gyro, i, num_modules);

      if (i == body_frame_module - 1) {
        u_t.tail(3) = gyro;
      }

      imus_ready[i] = true;
    }
};

void handle_joints(sensor_msgs::JointState msg) {
  double time = msg.header.stamp.toSec();
  VectorXd joints(num_modules);
  for (size_t i = 0; i < num_modules; i++) {
    joints(i) = msg.position[i];
    if (ekf.angles.size() > 0) {
      ekf.angles[i] = joints(i);
    }
  }
  if (!joints_ready) {
    u_t.segment(0, num_modules).setZero();
    joints_ready = true;
  } else {
    u_t.segment(0, num_modules) = (joints - prev_joints)/(time - prev_joint_time);
  }
  prev_joint_time = time;
  prev_joints = joints;
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
  ekf.Q.block(12, 12, num_modules, num_modules) *= 100;

  ekf.R = 0.5*MatrixXd::Identity(sensorlen, sensorlen);
  ekf.R.block(0, 0, 3*num_modules, 3*num_modules) /= 50;

  ekf.S_t = 0.001*MatrixXd::Identity(statelen - 1, statelen - 1);

  if (gait.compare("sidewind") == 0) {
    ekf.Q.block<3, 3>(3, 3) *= 100;
    ekf.R = 0.001*MatrixXd::Identity(sensorlen, sensorlen);
  }

  // Adjust covariances for virtual chassis estimator
  if (body_frame_module < 0) {
    if (gait.compare("roll") == 0) {
      ekf.Q.block<3, 3>(3, 3) *= 100;
      ekf.Q.block<3, 3>(6, 6) *= 100;
    }
    ekf.Q.block(12, 12, num_modules, num_modules) *= 10;
  }

  if (body_frame_module < 0) {
    u_t = VectorXd::Zero(num_modules);
  } else {
    u_t = VectorXd::Zero(num_modules + 3);
  }

  // Initialize pose messages
  head_frame.header.frame_id = "body_frame";
  head_frame.child_frame_id = "link0";

  body_frame.header.frame_id = "world";
  body_frame.child_frame_id = "body_frame";

  body_frame.transform.translation.x = 0;
  body_frame.transform.translation.y = 0;
  body_frame.transform.translation.z = 0;

  joint_pub = n.advertise<sensor_msgs::JointState>("/reusnake/joint_state", 1);
  pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/reusnake/head_pose", 2);

  vector<ros::Subscriber> imu_subs;
  ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/snake/joint_states", 1, handle_joints); 
  for (size_t i = 0; i < num_modules; i++) {
    if (i + 1 < 10) {
      imu_subs.push_back(n.subscribe<sensor_msgs::Imu>("/snake/sensors/SA00" + to_string(i + 1) + "__MoJo/imu", 3, handle_imu(i)));
    } else {
      imu_subs.push_back(n.subscribe<sensor_msgs::Imu>("/snake/sensors/SA0" + to_string(i + 1) + "__MoJo/imu", 3, handle_imu(i)));
    }
    imus_ready[i] = false;
  }

  tf2_ros::TransformBroadcaster pose_br;
  bool ready = false;
  ros::Rate r(50); 
  while (ros::ok()) {
    if (!ready) {
      ready = true;
      for (size_t i = 0; i < num_modules; i++) {
        if (!joints_ready || !imus_ready[i]) {
          ready = false;
          break;
        }
      }
      if (ready) {
        ekf.initialize(num_modules, body_frame_module, z_t, prev_joints);
      }
    }
    if (ready) {
      ekf.predict(u_t, dt);
      ekf.correct(z_t);   

      for (size_t i = 0; i < num_modules; i++) { 
        if (joint_state.position.size() <= i) { 
          joint_state.position.push_back(ekf.angles[i]);
        } else {
          joint_state.position[i] = ekf.angles[i];
        }
      }
      Vector4d q_t = get_q(ekf.x_t);

      // Filter estimates body frame orientation. Now calculate head orientation
      transformArray transforms = makeSEASnake(ekf.angles);
      Matrix4d vc = getSnakeVirtualChassis(transforms);
      makeVirtualChassisConsistent(ekf.vc, vc);
      ekf.vc = vc;

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

      head_pose.header.stamp = ros::Time::now();
      pose_pub.publish(head_pose);

      head_frame.header.stamp = ros::Time::now();
      pose_br.sendTransform(head_frame);
      body_frame.header.stamp = head_frame.header.stamp;
      pose_br.sendTransform(body_frame);
      joint_state.header.stamp = head_frame.header.stamp;
      joint_pub.publish(joint_state);
    }
    
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
