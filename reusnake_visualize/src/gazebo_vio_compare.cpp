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
#include <tf2_ros/transform_broadcaster.h>

using namespace Eigen;

static bool first_tf = true;
static bool first_gazebo = true;
static Quaterniond gazebo_unyaw;
static Vector3d gazebo_initial_pos;
static Quaterniond vo_unyaw;
static Vector3d vo_initial_pos;

static Quaterniond gazebo_rot;
static Quaterniond vo_rot;
static Vector3d gazebo_pos;
static Vector3d vo_pos;

Matrix3d rotX(double angle){
  
  // Calculate sin/cos
  double s = sin(angle);
  double c = cos(angle);
  
  // Create matrix
  Matrix3d rot;
  rot << 1, 0, 0,
         0, c, -s,
         0, s, c;
  
  // Return
  return rot;
}

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
        first_gazebo = false;
        Vector3d euler = gazebo_rot.toRotationMatrix().eulerAngles(2, 1, 0);
        gazebo_unyaw = rotZ(-euler(0));
        gazebo_initial_pos = gazebo_pos;
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

  ros::Subscriber gazebo_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, handle_gazebo);
  tf::TransformListener tf_listener;

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped gazebo_tf;
  geometry_msgs::TransformStamped vo_tf_pub;
  gazebo_tf.header.frame_id = "world";
  gazebo_tf.child_frame_id = "ground_truth";

  bool got_vo = false;

  ros::Rate r(50); // 50 hz
  while (ros::ok()) {
    if (!first_gazebo) {
      try {
        gazebo_tf.header.stamp = ros::Time::now();
        gazebo_tf.transform.translation.x = gazebo_pos(0);
        gazebo_tf.transform.translation.y = gazebo_pos(1);
        gazebo_tf.transform.translation.z = gazebo_pos(2);
        gazebo_tf.transform.rotation.w = gazebo_rot.w();
        gazebo_tf.transform.rotation.x = gazebo_rot.x();
        gazebo_tf.transform.rotation.y = gazebo_rot.y();
        gazebo_tf.transform.rotation.z = gazebo_rot.z();
        br.sendTransform(gazebo_tf);

        tf::StampedTransform vo_tf;
        tf_listener.lookupTransform("world", "snake_vio_pose", ros::Time(0), vo_tf);

        vo_rot.w() = vo_tf.getRotation().w();
        vo_rot.x() = vo_tf.getRotation().x();
        vo_rot.y() = vo_tf.getRotation().y();
        vo_rot.z() = vo_tf.getRotation().z();

        vo_pos(0) = vo_tf.getOrigin().x();
        vo_pos(1) = vo_tf.getOrigin().y();
        vo_pos(2) = vo_tf.getOrigin().z();

        if (!got_vo) {
          Vector3d euler = vo_rot.toRotationMatrix().eulerAngles(2, 1, 0);
          vo_unyaw = rotZ(-euler(0));
          vo_initial_pos = vo_pos;
          got_vo = true;
        }
        vo_rot = vo_unyaw*vo_rot;
        vo_pos = vo_unyaw*(vo_pos - vo_initial_pos);

        vo_tf_pub.header.stamp = ros::Time::now();
        vo_tf_pub.header.frame_id = "world";
        vo_tf_pub.child_frame_id = "kinematic_motion_model";
        vo_tf_pub.transform.translation.x = vo_pos(0);
        vo_tf_pub.transform.translation.y = vo_pos(1);
        vo_tf_pub.transform.translation.z = vo_pos(2);
        vo_tf_pub.transform.rotation.w = vo_rot.w();
        vo_tf_pub.transform.rotation.x = vo_rot.x();
        vo_tf_pub.transform.rotation.y = vo_rot.y();
        vo_tf_pub.transform.rotation.z = vo_rot.z();
        br.sendTransform(vo_tf_pub);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    }
      
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
