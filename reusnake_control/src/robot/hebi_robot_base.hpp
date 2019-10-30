#pragma once

#include "robot_base.hpp"

// hebi cpp api, need to be changed
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/group_info.hpp"
#include "hebi_cpp_api/lookup.hpp"

// ros
#include <ros/package.h>

using namespace Eigen;

namespace robot {
  
class HebiRobotBase : public RobotBase {

  public:
    HebiRobotBase();
    ~HebiRobotBase();

    bool setGains();
    void setupLogging();

    virtual void setCommand(const VectorXd* angles, const VectorXd* vels, const VectorXd* accels);
    virtual void saveCommand();
    virtual void loadCommand();
    virtual void freeze(); // stay where the robot was commanded
    virtual void sendCommand();

  protected:
    // we need hebi API to control hebi robot
    std::shared_ptr<hebi::Group> group_;
    std::shared_ptr<hebi::Group> log_group_input_;
    std::shared_ptr<hebi::Group> log_group_modules_;

    std::mutex hebi_fbk_lock_;

    // cmd used for control robot
    hebi::GroupCommand* cmd_;
    hebi::GroupCommand* saved_cmd_;
    hebi::GroupCommand* saved_stance_cmd_;   
};

}