#include "ros/ros.h"
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include "hebiros/FeedbackMsg.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SetFeedbackFrequencySrv.h"
#include "SnakeKinematics.cpp"
#include <iostream>

/* This file performs estimation by subscribing to feedback
 * from the hebiros node
 */

using namespace Eigen;
using namespace hebiros;

static const int feedback_freq = 100;
static const double dt = 1.0/feedback_freq;
static const double alpha = 0.02;

static geometry_msgs::TransformStamped pose;
static bool first = true;

Vector4d q_t(1, 0, 0, 0);

// Get state transition matrix for quaternion
void quaternion_stm(Matrix4d& stm, Vector3d& w_t, double dt) {
  Matrix4d omega; // skew-symmetric matrix used in orientation update
  omega <<            0, -w_t(0), -w_t(1), -w_t(2),
               w_t(0),         0,  w_t(2), -w_t(1),
               w_t(1), -w_t(2),         0,  w_t(0),
               w_t(2),  w_t(1), -w_t(0),         0;
  Matrix4d I = Matrix<double, 4, 4>::Identity();
  
  double wmag = w_t.norm(); // magnitude of angular velocity
  double s = 0.5*dt*wmag;

  if (s == 0) {
    stm = I;
  } else {
    stm = I*cos(s) + omega*sin(s)/wmag;
  }
}

void handle_feedback(FeedbackMsg msg) {
  if (first) {
    Vector3d a_grav_module(msg.accelerometer[0].x, msg.accelerometer[0].y, msg.accelerometer[0].z); // module 1 accelerometer value
    Vector3d a_grav(0, 0, 9.8); // gravitational acceleration in world frame
    Quaterniond q_m1;
    q_m1.setFromTwoVectors(a_grav_module, a_grav);

    // Set yaw to 0
    Vector3d ypr = q_m1.toRotationMatrix().eulerAngles(2, 1, 0);
    Matrix3d new_rot = rotZ(0)*rotY(ypr(1))*rotX(ypr(2));

    Quaterniond new_q(new_rot);

    q_t(0) = new_q.w();
    q_t(1) = new_q.x();
    q_t(2) = new_q.y();
    q_t(3) = new_q.z();

    first = false;
  }
  
  Vector3d w_t(msg.gyro[0].x, msg.gyro[0].y, msg.gyro[0].z); 
  Matrix4d stm;
  quaternion_stm(stm, w_t, dt);
  
  q_t = stm*q_t;

  Quaterniond q_m1_world(q_t(0), q_t(1), q_t(2), q_t(3));

  //Compute tilt correction quaternion based on accelerometer
  Vector3d a_t(msg.accelerometer[0].x,
               msg.accelerometer[0].y,
               msg.accelerometer[0].z); 
  a_t = q_m1_world.toRotationMatrix()*a_t/a_t.norm();
  Vector3d a_grav(0, 0, 1);
  double angle = acos(a_t.dot(a_grav));
  Vector3d axis = a_t.cross(a_grav);
  axis = axis/axis.norm();
  
  AngleAxisd angle_axis((1 - alpha)*angle, axis);
  Quaterniond tilt_corr(angle_axis);
  q_m1_world = tilt_corr*q_m1_world;

  q_t(0) = q_m1_world.w();
  q_t(1) = q_m1_world.x();
  q_t(2) = q_m1_world.y();
  q_t(3) = q_m1_world.z();

  q_t = q_t/q_t.norm();
  
  // Filter estimates module 1 orientation. Now calculate head orientation
  vector<double> angles(1);
  angles.push_back(msg.position[0]);
  transformArray transforms = makeUnifiedSnake(angles);
  // Orientation of module 1 wrt head
  Matrix3d R = transforms[1].block(0, 0, 3, 3);
  Quaterniond q_m1_head(R); 

  Quaterniond q_head = q_m1_world*q_m1_head.conjugate();

  pose.header.stamp = ros::Time::now();
  pose.transform.rotation.w = q_head.w();
  pose.transform.rotation.x = q_head.x();
  pose.transform.rotation.y = q_head.y();
  pose.transform.rotation.z = q_head.z();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "complementary_filter");
  ros::NodeHandle n;
  
  // Initialize pose message 
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "world";
  pose.child_frame_id = "complementary";
  pose.transform.translation.x = 1;
  pose.transform.translation.y = 0;
  pose.transform.translation.z = 0;
  
  pose.transform.rotation.w = 1;
  pose.transform.rotation.x = 0;
  pose.transform.rotation.y = 0;
  pose.transform.rotation.z = 0;
  
  // Initialize group using hebiros node
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>("hebiros/add_group_from_names");
  AddGroupFromNamesSrv add_group_srv;
  add_group_srv.request.group_name = "RUSNAKE";
  add_group_srv.request.names = {"RUSnake Module #13",
        "RUSnake Module #12",
        "RUSnake Module #11",
        "RUSnake Module #10",
        "RUSnake Module #9",
        "RUSnake Module #8",
        "RUSnake Module #6",
        "RUSnake Module #3",
        "RUSnake Module #2",
        "RUSnake Module #1"
};
  add_group_srv.request.families = {"*"};
  // Block until group is created
  if (!add_group_client.call(add_group_srv)) {
    cout << "Lookup of RUSNAKE failed.\n";  
    exit(1);
  } 
  
  // Set feedback frequency
  ros::ServiceClient set_freq_client = n.serviceClient<SetFeedbackFrequencySrv>("hebiros/set_feedback_frequency");
  SetFeedbackFrequencySrv set_freq_srv;
  set_freq_srv.request.feedback_frequency = feedback_freq;
  set_freq_client.call(set_freq_srv);

  ros::Subscriber feedback_sub = n.subscribe("/hebiros/RUSNAKE/feedback", 100, handle_feedback);

  tf2_ros::TransformBroadcaster pose_br;
  ros::Rate r(50);
  while (ros::ok()) {
    pose_br.sendTransform(pose);
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
