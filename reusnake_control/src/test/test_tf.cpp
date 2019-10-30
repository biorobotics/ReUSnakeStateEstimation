// This file helps to calcuate the transformation between mat6 joint 1 base motor to realsense camera mount
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "test_tf");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);
  tf::TransformBroadcaster _tf_broadcaster;
  tf::TransformListener listener;

  /* ROS loop */
  for (int publish_count = 0; nh.ok(); publish_count++)
  {    
    geometry_msgs::TransformStamped msg;
    Eigen::Matrix3d R;
    Eigen::Quaterniond q;
    double angle;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.child_frame_id = "m1_link";
    msg.transform.translation.x = 0.205681;
    msg.transform.translation.y = 0.118749;
    msg.transform.translation.z = 0;
    
    R.setZero();
    angle = 0.52359;  // tilt angle of realsense D435, measured in CAD
    R <<    cos(angle),      sin(angle),    0, 
           -sin(angle),      cos(angle),    0,
                     0,               0,    1;
    q = Eigen::Quaterniond(R);
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();
    _tf_broadcaster.sendTransform(msg);
  
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.child_frame_id = "d_link";
    msg.transform.translation.x = 0.05;
    msg.transform.translation.y = 0;
    msg.transform.translation.z = 0.27;  // translations from base to d_link, obtained in CAD
    Eigen::Matrix3d mat;
    mat.setZero();
    angle = 0.33;  // tilt angle of realsense D435, measured in CAD
    mat <<  1,           0,               0,
            0,  cos(angle),      sin(angle), 
            0, -sin(angle),      cos(angle);

    Eigen::Matrix3d mat2;
    mat2.setZero();
    mat2 <<  0,      0,       1, 
            -1,      0,       0,
             0,     -1,       0;
    // from d_link to base_link, first rotate around x using mat
    // then change direction of axes using mat2. This order is important
    q = Eigen::Quaterniond(mat2*mat);  
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();
    _tf_broadcaster.sendTransform(msg);


    // tf::StampedTransform transform;
    // listener.lookupTransform("/d_link", "/m1_link", ros::Time(0), transform);

    
    // Eigen::Vector3d vec(transform.getOrigin().x(),
    //                     transform.getOrigin().y(),
    //                     transform.getOrigin().z());
    // std::cout << vec << std::endl;

    // d_link to m1_link rotation
      // 0.819752  0.472356 -0.325348
      // 0.49934 -0.866616  0.000868
      // -0.281124 -0.163412 -0.946104
    //
    // q = Eigen::Quaterniond(0.503,0.705,-0.407,0.291); 
    // std::cout << q.toRotationMatrix() << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}