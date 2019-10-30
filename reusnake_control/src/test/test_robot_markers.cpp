#include <map>
#include <string>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <visualization_msgs/MarkerArray.h>
#include "robot_markers/builder.h"
#include <ros/package.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_markers_demo");
  ros::NodeHandle nh;
  ros::Publisher marker_arr_pub =
      nh.advertise<visualization_msgs::MarkerArray>("robot", 2);

  ros::Duration(0.5).sleep();
  ros::Rate loop_rate(50);

  geometry_msgs::Pose pose;
  std::map<std::string, double> joint_positions;
  urdf::Model model;
  //ROS_INFO("model inited"); 
  //model.initParam("robot_description");
  std::string urdf_path = ros::package::getPath("titan6_general_cpp") + "/urdf/m6.urdf";
	model.initFile(urdf_path);
  
  ROS_INFO("model loaded"); 
  robot_markers::Builder builder(model);
  builder.Init();
  ROS_INFO("builder inited"); 

  // Robot 1: Default configuration, purple.
  visualization_msgs::MarkerArray robot1;


  for (int i = 0; i< 6; i++) {
    joint_positions["base"+std::to_string(i+1)] =0;
    joint_positions["shoulder"+std::to_string(i+1)] = 0;
    joint_positions["elbow"+std::to_string(i+1)] = 0;
  }

  /* ROS loop */
  for (int publish_count = 0; nh.ok(); publish_count++)
  {
    builder.SetNamespace("robot");
    builder.SetFrameId("world");
    ROS_INFO("pose %4.3f %4.3f", pose.position.x, pose.position.y); 
    builder.SetPose(pose);
    builder.SetJointPositions(joint_positions);
    ROS_INFO("builder set parameters");
    builder.Build(&robot1);

    ROS_INFO("robot built");
    marker_arr_pub.publish(robot1);
    robot1.markers.clear();  
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}