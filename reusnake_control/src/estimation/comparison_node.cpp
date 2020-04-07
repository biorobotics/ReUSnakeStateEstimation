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

void signalHandler( int signum ) {
  file.close();
  file1.close();
  file2.close();
  file3.close();
  exit(signum);
}

Vector3d quaternion_to_euler(Quaterniond& q, Vector3d& prev_ypr) {
  /*
  Vector3d ret = Vector3d::Zero();
  Matrix3d R = q.toRotationMatrix();
  if (R(1, 2) != 0 || R(0, 2) != 0) {
    ret(0) = atan2(R(1, 2), R(0, 2));
    if (sin(prev_ypr(1)) < 0) {
      if (ret(0) < 0) {
        ret(0) += M_PI;
      } else {
        ret(0) -= M_PI;
      }
    }
  } else {
    // Pure z rotation
    ret(0) = atan2(R(1, 0), R(0, 0));
    return ret;
  }

  double stheta;
  if (sin(ret(0)) != 0) {
    stheta = R(1, 2)/sin(ret(0));
  } else {
    stheta = R(0, 2)/cos(ret(0));
  }
  ret(1) = atan2(stheta, R(2, 2));
  ret(2) = atan2(R(2, 1), -R(2, 0));
    
  if (stheta < 0) {
    if (ret(2) < 0) {
      ret(2) += M_PI;
    } else {
      ret(2) -= M_PI;
    }
  }

  for (size_t i = 0; i < 3; i++) {
    double diff = ret(i) - prev_ypr(i);
    if (diff > 5) {
      ret(i) -= 2*M_PI;
    } else if (diff < -5) {
      ret(i) += 2*M_PI;
    }
  }

  return ret;
  */
  Vector3d ret = Vector3d::Zero();
  Matrix3d R = q.toRotationMatrix();
  ret(0) = atan2(R(1, 0), R(0, 0));
  if (cos(prev_ypr(1)) < 0) {
    if (ret(0) < 0) {
      ret(0) += M_PI;
    } else {
      ret(0) -= M_PI;
    }
  }

  double ctheta;
  if (sin(ret(0)) != 0) {
    ctheta = R(1, 0)/sin(ret(0));
  } else {
    ctheta = R(0, 0)/cos(ret(0));
  }
  ret(1) = atan2(-R(2, 0), ctheta);
  ret(2) = atan2(R(2, 1), R(2, 2));
    
  if (ctheta < 0) {
    if (ret(2) < 0) {
      ret(2) += M_PI;
    } else {
      ret(2) -= M_PI;
    }
  }

  for (size_t i = 0; i < 3; i++) {
    double diff = ret(i) - prev_ypr(i);
    if (diff > 5) {
      ret(i) -= 2*M_PI;
    } else if (diff < -5) {
      ret(i) += 2*M_PI;
    }
  }

  return ret;
}

Matrix3d rotX(double angle){
  
  // Calculate sin/cos
  double s = sin(angle);
  double c = cos(angle);
  
  // Create matrix
  Matrix3d rot;
  rot << 1,  0,  0,
         0,  c, -s,
         0,  s,  c; 
  
  // Return
  return rot;
}

Matrix3d rotY(double angle){
  
  // Calculate sin/cos
  double s = sin(angle);
  double c = cos(angle);
  
  // Create matrix
  Matrix3d rot;
  rot << c,  0,  s,
         0,  1,  0,
        -s,  0,  c;
  
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
  
int main(int argc, char **argv) {
  ros::init(argc, argv, "comparison");
  ros::NodeHandle n;

  tf::TransformListener tf_listener;
  
  ros::Rate r(50); // 50 hz

  bool first = true;
  Quaterniond vins_initial_rot;
  Quaterniond estimator_initial_rot;
  Vector3d vins_initial_pos;
  Vector3d estimator_initial_pos;

  file.open("vins_rpy.txt");
  file1.open("estimator_rpy.txt");
  file2.open("vins_position.txt");
  file3.open("estimator_position.txt");

  signal(SIGINT, signalHandler);

  Vector3d vins_prev_ypr = Vector3d::Zero();
  Vector3d estimator_prev_ypr = Vector3d::Zero();

  while (ros::ok()) {
    try {
      tf::StampedTransform vins;
      tf::StampedTransform estimator;
      tf_listener.lookupTransform("body", "world",  
                                  ros::Time(0), vins);
      tf_listener.lookupTransform("link0", "world",  
                                  ros::Time(0), estimator);
      
      Quaterniond vins_rot;
      Quaterniond estimator_rot;

      vins_rot.w() = vins.getRotation().w();
      vins_rot.x() = vins.getRotation().x(),
      vins_rot.y() = vins.getRotation().y();
      vins_rot.z() = vins.getRotation().z(),
      
      estimator_rot.w() = estimator.getRotation().w();
      estimator_rot.x() = estimator.getRotation().x(),
      estimator_rot.y() = estimator.getRotation().y();
      estimator_rot.z() = estimator.getRotation().z();

      Vector3d vins_pos(vins.getOrigin().x(),
                       vins.getOrigin().y(),
                       vins.getOrigin().z());
      Vector3d estimator_pos(estimator.getOrigin().x(), estimator.getOrigin().y(),
                              estimator.getOrigin().z());

      if (first) {
        first = false;
        Vector3d vins_ypr = quaternion_to_euler(vins_rot, vins_prev_ypr);
        Vector3d estimator_ypr = quaternion_to_euler(estimator_rot, estimator_prev_ypr);

        vins_initial_rot = rotZ(vins_ypr(0));
        estimator_initial_rot = rotZ(estimator_ypr(0));

        vins_initial_pos = vins_pos;
        estimator_initial_pos = estimator_pos;
      }
  
      Quaterniond vins_rot_diff = vins_initial_rot.conjugate()*vins_rot;
      Quaterniond estimator_rot_diff = estimator_initial_rot.conjugate()*estimator_rot;

      vins_pos = vins_initial_rot.conjugate()*(vins_pos - vins_initial_pos);
      estimator_pos = estimator_initial_rot.conjugate()*(estimator_pos - estimator_initial_pos);

      if (vins_pos.norm() < 10) {
        Vector3d vins_ypr = quaternion_to_euler(vins_rot_diff, vins_prev_ypr);
        Vector3d estimator_ypr = quaternion_to_euler(estimator_rot_diff, estimator_prev_ypr);
        vins_prev_ypr = vins_ypr;
        estimator_prev_ypr = estimator_ypr;

        file << vins_ypr(2) << " " << vins_ypr(1) << " " << vins_ypr(0) << "\n";
        file1 << estimator_ypr(2) << " " << estimator_ypr(1) << " " << estimator_ypr(0) << "\n";
        file2 << vins_pos(0) << " " << vins_pos(1) << " " << vins_pos(2) << "\n";
        file3 << estimator_pos(0) << " " << estimator_pos(1) << " " << estimator_pos(2) << "\n";
      } else {
        file.close();
        file1.close();
        file2.close();
        file3.close();
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
