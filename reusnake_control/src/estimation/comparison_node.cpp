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
  
  ros::Rate r(50); // 50 hz

  bool first = true;
  Quaterniond vins_initial_pose;
  Quaterniond estimator_initial_pose;
  while (ros::ok()) {
    try {
      tf::StampedTransform vins;
      tf::StampedTransform estimator;
      tf_listener.lookupTransform("body", "world",  
                                  ros::Time(0), vins);
      tf_listener.lookupTransform("link0", "world",  
                                  ros::Time(0), vins);
      
      Quaterniond vins_pose;
      Quaterniond estimator_pose;

      vins_pose.w() = vins.getRotation().w();
      vins_pose.x() = vins.getRotation().x(),
      vins_pose.y() = vins.getRotation().y();
      vins_pose.z() = vins.getRotation().z(),
      
      estimator_pose.w() = estimator.getRotation().w();
      estimator_pose.x() = estimator.getRotation().x(),
      estimator_pose.y() = estimator.getRotation().y();
      estimator_pose.z() = estimator.getRotation().z();

      if (first) {
        first = false;
        vins_initial_pose = vins_pose;
        estimator_initial_pose = estimator_pose;
      }
  
      Quaterniond vins_disp = vins_initial_pose.conjugate()*vins_pose;
      Quaterniond estimator_disp = estimator_initial_pose.conjugate()*estimator_pose;

      Quaterniond err_q = vins_disp.conjugate()*estimator_disp;
      Vector3d ypr = err_q.toRotationMatrix().eulerAngles(2, 1, 0);
     
      Vector3d err_pos(vins.getOrigin().x(),
                       vins.getOrigin().y(),
                       vins.getOrigin().z());
      if (err_pos.norm() < 10) {
        printf("Difference between VINS and VC estimator - Roll: %lf Pitch: %lf Yaw: %lf\n",
               ypr(2), ypr(1), ypr(0));
      } else {
        printf("VINS unstable. Massive position error.\n");
        first = true;
      }

      geometry_msgs::Vector3 rpy_err;
      rpy_err.x = ypr(2);
      rpy_err.y = ypr(1);
      rpy_err.z = ypr(0);
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
