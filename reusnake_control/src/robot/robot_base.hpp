#pragma once
// Eigen
#include <Eigen/Dense>
// STL
#include <memory>
#include <set>
#include <chrono>
#include <map>
// MR
#include "../mr/modern_robotics.h"

// hebi useful trajectory tool, TODO: remove this from robot_base because robotbase should be decoupled from hebi API ( but we may keep using hebi trajectory)
#include "hebi_cpp_api/trajectory.hpp"

// add ROS to visualize orientation
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

using namespace Eigen;

namespace robot {
  
class RobotBase {
 public:
  RobotBase();
  // RobotBase(const RobotParameters& robot_params);
  virtual ~RobotBase();

  bool init();

  // these functions are different for different derived class. For example, M6 and N6 use hebiModules, but O6 uses some new modules.
  virtual void setCommand(const VectorXd* angles, const VectorXd* vels, const VectorXd* accels) = 0;
  virtual void saveCommand() = 0;
  virtual void loadCommand() = 0;
  // Actually send the command to the robot.
  // always call this in the main loop to avoid side effects (robot_base's member class never call this directly)
  virtual void freeze() = 0;
  virtual void sendCommand() = 0;

  // gaits
  void gait_sidewinding(double t);

 protected:
  int numModules;

  ros::NodeHandle nh;
  sensor_msgs::Imu imu_msg;
  ros::Publisher imu_pub; // test my estimator imu
  // tf::TransformBroadcaster _tf_broadcaster;

  // time variables to calculate time difference between feedbacks
  std::chrono::time_point<std::chrono::steady_clock> last_fbk_time_;
  std::chrono::time_point<std::chrono::steady_clock> curr_fbk_time_;
  std::chrono::duration<double, std::ratio<1>> fbk_dt_;

  // Allow Eigen member variables:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace robot