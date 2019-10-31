#pragma once

// std
#include <Eigen/Dense>
#include <memory>
#include <set>
#include <chrono>
#include <thread>
#include <map>

#include "hebi_robot_base.hpp"

// ROS
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>


namespace robot {

class Reusnake : public HebiRobotBase {
  public:
    static std::unique_ptr<Reusnake> create(ros::NodeHandle &_nh);

    ~Reusnake();
    Reusnake(std::shared_ptr<hebi::Group> group, ros::NodeHandle &_nh);
  

  private:
      
};

// class N6Base : public RobotBase {
//  public:
//   N6Base();
//   N6Base(const RobotParameters& robot_params);

// };
}