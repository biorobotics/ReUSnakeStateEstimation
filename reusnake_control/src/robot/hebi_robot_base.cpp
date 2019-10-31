#include "hebi_robot_base.hpp"

namespace robot {

HebiRobotBase::HebiRobotBase() 
{
  numModules = 13 ;
  cmd_ = new hebi::GroupCommand(numModules);
  saved_cmd_ = new hebi::GroupCommand(numModules);
  saved_stance_cmd_ = new hebi::GroupCommand(numModules);
}


void HebiRobotBase::setCommand(const VectorXd* angles, const VectorXd* vels, const VectorXd* torques)
{
  if (angles != nullptr)
  {
    assert(angles->size() == numModules);
    for (int i = 0; i < numModules; ++i)
      (*cmd_)[i].actuator().position().set((*angles)[i]);
  }
  if (vels != nullptr)
  {
    assert(vels->size() == numModules);
    for (int i = 0; i < numModules; ++i)
      (*cmd_)[i].actuator().velocity().set((*vels)[i]);
  }
  if (torques != nullptr)
  {
    assert(torques->size() == numModules);
    for (int i = 0; i < numModules; ++i)
      (*cmd_)[i].actuator().effort().set((*torques)[i]);
  }
}

void HebiRobotBase::saveCommand()
{
  // std::cout<<"save cmd_"<<std::endl;
  for (int i = 0; i < numModules; i++)
  {
    (*saved_cmd_)[i].actuator().position().set((*cmd_)[i].actuator().position().get());
    (*saved_cmd_)[i].actuator().velocity().set((*cmd_)[i].actuator().velocity().get());
    (*saved_cmd_)[i].actuator().effort().set((*cmd_)[i].actuator().effort().get());
  } 
}

void HebiRobotBase::loadCommand()
{
  // std::cout<<"load cmd_"<<std::endl;
  for (int i = 0; i < numModules; i++)
  {
    (*cmd_)[i].actuator().position().set((*saved_cmd_)[i].actuator().position().get());
    (*cmd_)[i].actuator().velocity().set((*saved_cmd_)[i].actuator().velocity().get());
    (*cmd_)[i].actuator().effort().set((*saved_cmd_)[i].actuator().effort().get());
  }
}

void HebiRobotBase::freeze()
{
  loadCommand();
  saveCommand();
  sendCommand();
}

void HebiRobotBase::sendCommand() {
  group_->sendCommand(*cmd_);
}

bool HebiRobotBase::setGains()
{
  if (!group_)
    return false;

  // hebi::GroupCommand gains(group_->size());
  // std::string gains_file = ros::package::getPath("titan6_general_cpp") + std::string("/resources/gains") + std::to_string(group_->size()) + ".xml";
  // std::cout << "Loading gains from: " << gains_file << std::endl;
  // bool success = gains.readGains(gains_file);
  // return success && group_->sendCommandWithAcknowledgement(gains, 4000);
  return true;
}

HebiRobotBase::~HebiRobotBase() {
    if (group_) {
      group_->setFeedbackFrequencyHz(0);
      group_->clearFeedbackHandlers();
    }
}

} // end of namespace robot
