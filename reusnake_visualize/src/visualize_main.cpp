// stl
#include <iostream>
#include <set>
#include <chrono>
#include <thread>
#include <atomic>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// visualizer
#include "robot_markers/builder.h"

// include modernrobotics to do some frame transformation
#include "mr/modern_robotics.h"

// for visualize robot in rviz robomarker
robot_markers::Builder* builder;
visualization_msgs::MarkerArray snake_vis_array;
ros::Publisher marker_arr_pub;

// body pose of the snake
geometry_msgs::Pose body_pose;

// tf related

void joint_state_callback(const sensor_msgs::JointStateConstPtr& joint_state) 
{
  std::map<std::string, double> joint_positions;
  for (int i = 0; i< joint_state->position.size(); i++) {
    // TODO: use name from message
    joint_positions["joint"+std::to_string(i+1)] = joint_state->position[i];
  }

  builder->SetNamespace("reusnake");
  builder->SetFrameId("world");
  builder->Build(&snake_vis_array);
  // notice that this body pose is the pose between world frame and the root of the robot (link0 for reusnake)
  builder->SetPose(body_pose);
  builder->SetJointPositions(joint_positions);
  marker_arr_pub.publish(snake_vis_array);
  // clear array for next time use
  snake_vis_array.markers.clear();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "reusnake_visualizer");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);


  // publisher 
  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformListener tf_listener;

  // robomarker visualizer
  // robot marker 
  marker_arr_pub = nh.advertise<visualization_msgs::MarkerArray>("/reusnake/model", 100);
  urdf::Model model;
  model.initFile(ros::package::getPath("reusnake_visualize") + "/urdf/sea_snake.urdf");
  builder = new robot_markers::Builder(model);
  builder->Init();

  //
  ros::Subscriber joint_state_sub = nh.subscribe("/reusnake/joint_state", 1000, joint_state_callback);
  

  /* ROS loop */
  for (int publish_count = 0; nh.ok(); publish_count++)
  {    
    tf::StampedTransform transform;
    try{
      tf_listener.lookupTransform("link0", "world",  
                                ros::Time(0), transform);
      body_pose.position.x = transform.getOrigin().x();
      body_pose.position.y = transform.getOrigin().y();
      body_pose.position.z = transform.getOrigin().z();    
      body_pose.orientation.w = transform.getRotation().w();
      body_pose.orientation.x = transform.getRotation().x();
      body_pose.orientation.y = transform.getRotation().y();
      body_pose.orientation.z = transform.getRotation().z();                          
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;      
}