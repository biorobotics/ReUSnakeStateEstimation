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
  Quaterniond initial_err = Quaterniond::Identity();
  while (ros::ok()) {
    try {
      tf::StampedTransform err;
      tf_listener.lookupTransform("link0", "body",  
                                  ros::Time(0), err);
      Quaterniond err_q(err.getRotation().w(),
                        err.getRotation().x(),
                        err.getRotation().y(),
                        err.getRotation().z());
      err_q = initial_err.conjugate()*err_q;

      if (first) {
        first = false;
        initial_err = err_q;
      }

      Vector3d ypr = err_q.toRotationMatrix().eulerAngles(2, 1, 0);
     
      Vector3d err_pos(err.getOrigin().x(),
                       err.getOrigin().y(),
                       err.getOrigin().z());
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
