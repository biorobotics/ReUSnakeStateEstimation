#include "ros/ros.h"
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

using namespace Eigen;

int main(int argc, char **argv) {
  ros::init(argc, argv, "comparison");
  ros::NodeHandle n;

  ros::Publisher err_pub = n.advertise<geometry_msgs::Vector3>("/comparison/rpy_error", 100);

  tf::TransformListener tf_listener;
  
  ros::Rate r(50); // 10 hz
  while (ros::ok()) {
    try {
      tf::StampedTransform err;
      tf_listener.lookupTransform("head_vc", "body",  
                                  ros::Time(0), err);
      Quaterniond err_q(err.getOrigin().w(), err.getOrigin().x(), err.getOrigin().y(), err.getOrigin().z());
      Vector3d rpy = err_q.toRotationMatrix().eulerAngles(2, 1, 0);
     
      printf("Difference between VINS and VC estimator - Roll: %lf Pitch: %lf Yaw: %lf\n",
             rpy(2), rpy(1), rpy(0));

      geometry_msgs::Vector3 rpy_err;
      rpy_err.x = rpy(0);
      rpy_err.y = rpy(1);
      rpy_err.z = rpy(2);
      err_pub.publish(rpy_err);
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
