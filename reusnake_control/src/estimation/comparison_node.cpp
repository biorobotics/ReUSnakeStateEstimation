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
#include <fstream>
#include <cmath>
#include <signal.h>

using namespace Eigen;

static std::ofstream file;
static std::ofstream file1;
static std::ofstream file2;
static std::ofstream file3;
static std::ofstream file4;

void signalHandler( int signum ) {
  file.close();
  file1.close();
  file2.close();
  file3.close();
  file4.close();
  exit(signum);
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
  
int main(int argc, char **argv) {
  ros::init(argc, argv, "comparison");
  ros::NodeHandle n;

  tf::TransformListener tf_listener;
  
  ros::Rate r(50); // 50 hz

  bool first = true;
  Quaterniond vins_initial_rot;
  Quaterniond estimator_initial_rot;
  Quaterniond complementary_initial_rot;
  Vector3d vins_initial_pos;
  Vector3d estimator_initial_pos;

  file.open("vins_q.txt");
  file1.open("estimator_q.txt");
  file2.open("vins_position.txt");
  file3.open("estimator_position.txt");
  file4.open("complementary_q.txt");

  signal(SIGINT, signalHandler);

  while (ros::ok()) {
    try {
      tf::StampedTransform vins;
      tf::StampedTransform estimator;
      tf::StampedTransform complementary;
      tf_listener.lookupTransform("world", "body",  
                                  ros::Time(0), vins);
      tf_listener.lookupTransform("world", "link0",  
                                  ros::Time(0), estimator);
      tf_listener.lookupTransform("world", "complementary",  
                                  ros::Time(0), complementary);
      
      Quaterniond vins_rot;
      Quaterniond estimator_rot;
      Quaterniond complementary_rot;

      vins_rot.w() = vins.getRotation().w();
      vins_rot.x() = vins.getRotation().x(),
      vins_rot.y() = vins.getRotation().y();
      vins_rot.z() = vins.getRotation().z(),
      
      estimator_rot.w() = estimator.getRotation().w();
      estimator_rot.x() = estimator.getRotation().x(),
      estimator_rot.y() = estimator.getRotation().y();
      estimator_rot.z() = estimator.getRotation().z();

      complementary_rot.w() = complementary.getRotation().w();
      complementary_rot.x() = complementary.getRotation().x(),
      complementary_rot.y() = complementary.getRotation().y();
      complementary_rot.z() = complementary.getRotation().z();

      Vector3d vins_pos(vins.getOrigin().x(),
                       vins.getOrigin().y(),
                       vins.getOrigin().z());
      Vector3d estimator_pos(estimator.getOrigin().x(), estimator.getOrigin().y(),
                              estimator.getOrigin().z());

      if (first) {
        first = false;
        Vector3d vins_ypr = vins_rot.toRotationMatrix().eulerAngles(2, 1, 0);
        Vector3d estimator_ypr = estimator_rot.toRotationMatrix().eulerAngles(2, 1, 0);
        Vector3d complementary_ypr = complementary_rot.toRotationMatrix().eulerAngles(2, 1, 0);
        vins_initial_rot = rotZ(vins_ypr(0));
        estimator_initial_rot = rotZ(estimator_ypr(0));
        complementary_initial_rot = rotZ(complementary_ypr(0));

        vins_initial_pos = vins_pos;
        estimator_initial_pos = estimator_pos;
      }
  
      vins_rot = vins_initial_rot.conjugate()*vins_rot;
      estimator_rot = estimator_initial_rot.conjugate()*estimator_rot;
      complementary_rot = complementary_initial_rot.conjugate()*complementary_rot;

      vins_pos = vins_initial_rot.conjugate()*(vins_pos - vins_initial_pos);
      estimator_pos = estimator_initial_rot.conjugate()*(estimator_pos - estimator_initial_pos);

      if (vins_pos.norm() < 10) {
        file << vins_rot.w() << " " << vins_rot.x() << " " << vins_rot.y() << " " << vins_rot.z() << "\n";
        file1 << estimator_rot.w() << " " << estimator_rot.x() << " " << estimator_rot.y() << " " << estimator_rot.z() << "\n";
        file2 << vins_pos(0) << " " << vins_pos(1) << " " << vins_pos(2) << "\n";
        file3 << estimator_pos(0) << " " << estimator_pos(1) << " " << estimator_pos(2) << "\n";
        file4 << complementary_rot.w() << " " << complementary_rot.x() << " " << complementary_rot.y() << " " << complementary_rot.z() << "\n";
      } else {
        file.close();
        file1.close();
        file2.close();
        file3.close();
        file4.close();
        printf("VINS unstable. Massive position error.\n");
        first = true;
        exit(1);
      }
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
