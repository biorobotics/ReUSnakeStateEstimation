#include "ros/ros.h"
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <iostream> 
#include <fstream>
#include <cmath>
#include <signal.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace Eigen;

static std::ofstream file;
static std::ofstream file1;
static std::ofstream file2;
static std::ofstream file3;

static bool first_tf = true;
static bool first_gazebo = true;
static Quaterniond gazebo_unyaw;
static Quaterniond estimator_unyaw;
static Vector3d gazebo_initial_pos;
static Vector3d estimator_initial_pos;

static Quaterniond gazebo_rot;
static Quaterniond estimator_rot;
static Vector3d gazebo_pos;
static Vector3d estimator_pos;

Matrix3d rotY(double angle){
  
  // Calculate sin/cos
  double s = sin(angle);
  double c = cos(angle);
  
  // Create matrix
  Matrix3d rot;
  rot << c, 0, s,
         0, 1, 0,
         -s, 0, c;
  
  // Return
  return rot;
}

Matrix3d rotZ(double angle){
  
  // Calculate sin/cos
  double s = sin(angle);
  double c = cos(angle);
  
  // Create matrix
  Matrix3d rot;
  rot << c, -s,  0,
         s,  c,  0,
         0,  0,  1;
  
  // Return
  return rot;
}

void signalHandler( int signum ) {
  file.close();
  file1.close();
  file2.close();
  file3.close();
  exit(signum);
}

void handle_gazebo(gazebo_msgs::LinkStates msg) {
  for (size_t i = 0; i < msg.pose.size(); i++) {
    if (msg.name[i] == "robot::kdl_dummy_root") {
      gazebo_rot.w() = msg.pose[i].orientation.w;
      gazebo_rot.x() = msg.pose[i].orientation.x;
      gazebo_rot.y() = msg.pose[i].orientation.y;
      gazebo_rot.z() = msg.pose[i].orientation.z;

      gazebo_pos(0) = msg.pose[i].position.x;
      gazebo_pos(1) = msg.pose[i].position.y;
      gazebo_pos(2) = msg.pose[i].position.z;

      gazebo_rot = gazebo_rot*rotY(-M_PI/2);

      if (first_gazebo) {
        Vector3d gazebo_ypr = gazebo_rot.toRotationMatrix().eulerAngles(2, 1, 0);
        gazebo_unyaw = rotZ(-gazebo_ypr(0));

        gazebo_initial_pos = gazebo_pos;
        first_gazebo = false;
      }
      gazebo_rot = gazebo_unyaw*gazebo_rot;
      gazebo_pos = gazebo_unyaw*(gazebo_pos - gazebo_initial_pos);
      return;
    }
  }
}
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "comparison");
  ros::NodeHandle n;

  file.open("gazebo_q.txt");
  file1.open("estimator_q.txt");
  file2.open("gazebo_position.txt");
  file3.open("estimator_position.txt");

  ros::Subscriber gazebo_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, handle_gazebo);
  tf::TransformListener tf_listener;

  signal(SIGINT, signalHandler);

  tf2_ros::TransformBroadcaster gazebo_br;
  geometry_msgs::TransformStamped gazebo_tf;
  gazebo_tf.header.frame_id = "world";
  gazebo_tf.child_frame_id = "gazebo_root";

  ros::Rate r(50); // 50 hz
  while (ros::ok()) {
    try {
      gazebo_tf.header.stamp = ros::Time::now();
      gazebo_tf.transform.translation.x = gazebo_pos(0);
      gazebo_tf.transform.translation.y = gazebo_pos(1);
      gazebo_tf.transform.translation.z = gazebo_pos(2);
      gazebo_tf.transform.rotation.w = gazebo_rot.w();
      gazebo_tf.transform.rotation.x = gazebo_rot.x();
      gazebo_tf.transform.rotation.y = gazebo_rot.y();
      gazebo_tf.transform.rotation.z = gazebo_rot.z();
      gazebo_br.sendTransform(gazebo_tf);
      tf::StampedTransform estimator_tf;
      tf_listener.lookupTransform("world", "link0", ros::Time(0), estimator_tf);

      estimator_rot.w() = estimator_tf.getRotation().w();
      estimator_rot.x() = estimator_tf.getRotation().x();
      estimator_rot.y() = estimator_tf.getRotation().y();
      estimator_rot.z() = estimator_tf.getRotation().z();

      estimator_pos(0) = estimator_tf.getOrigin().x();
      estimator_pos(1) = estimator_tf.getOrigin().y();
      estimator_pos(2) = estimator_tf.getOrigin().z();
      if (first_tf) {
        Vector3d estimator_ypr = estimator_rot.toRotationMatrix().eulerAngles(2, 1, 0);
        estimator_unyaw = rotZ(-estimator_ypr(0));

        estimator_initial_pos = estimator_pos;
        first_tf = false;
      }
      estimator_rot = estimator_unyaw*estimator_rot;
      estimator_pos = estimator_unyaw*(estimator_pos - estimator_initial_pos);

      file << gazebo_rot.w() << " " << gazebo_rot.x() << " " << gazebo_rot.y() << " " << gazebo_rot.z() << "\n";
      file1 << estimator_rot.w() << " " << estimator_rot.x() << " " << estimator_rot.y() << " " << estimator_rot.z() << "\n";
      file2 << gazebo_pos(0) << " " << gazebo_pos(1) << " " << gazebo_pos(2) << "\n";
      file3 << estimator_pos(0) << " " << estimator_pos(1) << " " << estimator_pos(2) << "\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
      
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
