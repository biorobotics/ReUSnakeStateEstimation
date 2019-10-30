#include "hebi_robot_base.hpp"

namespace robot {

HebiRobotBase::HebiRobotBase() 
  :cmd_(14), saved_cmd_(14), saved_stance_cmd_(14) {
    numModules = 14;
}


void HebiRobotBase::setCommand(const VectorXd* angles, const VectorXd* vels, const VectorXd* torques)
{
  if (angles != nullptr)
  {
    assert(angles->size() == numModules);
    for (int i = 0; i < numModules; ++i)
      cmd_[i].actuator().position().set((*angles)[i]);
  }
  if (vels != nullptr)
  {
    assert(vels->size() == numModules);
    for (int i = 0; i < numModules; ++i)
      cmd_[i].actuator().velocity().set((*vels)[i]);
  }
  if (torques != nullptr)
  {
    assert(torques->size() == numModules);
    for (int i = 0; i < numModules; ++i)
      cmd_[i].actuator().effort().set((*torques)[i]);
  }
}

void HebiRobotBase::saveCommand()
{
  // std::cout<<"save cmd_"<<std::endl;
  for (int i = 0; i < numModules; i++)
  {
    saved_cmd_[i].actuator().position().set(cmd_[i].actuator().position().get());
    saved_cmd_[i].actuator().velocity().set(cmd_[i].actuator().velocity().get());
    saved_cmd_[i].actuator().effort().set(cmd_[i].actuator().effort().get());
  } 
}

void HebiRobotBase::loadCommand()
{
  // std::cout<<"load cmd_"<<std::endl;
  for (int i = 0; i < numModules; i++)
  {
    cmd_[i].actuator().position().set(saved_cmd_[i].actuator().position().get());
    cmd_[i].actuator().velocity().set(saved_cmd_[i].actuator().velocity().get());
    cmd_[i].actuator().effort().set(saved_cmd_[i].actuator().effort().get());
  }
}

void HebiRobotBase::freeze()
{
  loadCommand();
  saveCommand();
  sendCommand();
}

void HebiRobotBase::sendCommand() {
  group_->sendCommand(cmd_);
}

bool HebiRobotBase::setGains()
{
  if (!group_)
    return false;

  hebi::GroupCommand gains(group_->size());
  std::string gains_file = ros::package::getPath("titan6_general_cpp") + std::string("/resources/gains") + std::to_string(group_->size()) + ".xml";
  std::cout << "Loading gains from: " << gains_file << std::endl;
  bool success = gains.readGains(gains_file);
  return success && group_->sendCommandWithAcknowledgement(gains, 4000);
}

void HebiRobotBase::setupLogging() {
  // Set up logging if enabled:
  // if (log_group_input_ || log_group_modules_)
  //   std::cout << "Logging to 'logs' directory at " << robot_params_ -> low_log_frequency_hz << "hz with bursts of " << robot_params_ -> high_log_frequency_hz << " hz every 30 minutes." << std::endl;

  // std::string log_name_base;
  // {
  //   char standard_file_name[40];
  //   std::time_t now = std::time(nullptr);
  //   std::tm* now_tm = std::localtime(&now);
  //   std::snprintf(standard_file_name, 40, "log_file_%04d-%02d-%02d_%02d.%02d.%02d",
  //                 1900 + now_tm->tm_year, 1 + now_tm->tm_mon, now_tm->tm_mday, now_tm->tm_hour,
  //                 now_tm->tm_min, now_tm->tm_sec);
  //   log_name_base = std::string(standard_file_name);
  // }

  // if (log_group_input_)
  // {
  //   log_group_input_->setFeedbackFrequencyHz(robot_params_ -> low_log_frequency_hz);
  //   log_group_input_->startLog("logs", log_name_base + "-IO.hebilog");
  // }
  // if (log_group_modules_)
  // {
  //   log_group_modules_->setFeedbackFrequencyHz(robot_params_ -> low_log_frequency_hz);
  //   log_group_modules_->startLog("logs", log_name_base + "-HEX.hebilog");

  // }
}

HebiRobotBase::~HebiRobotBase() {
    if (group_) {
      group_->setFeedbackFrequencyHz(0);
      group_->clearFeedbackHandlers();
    }

    // if (log_group_input_) {
    //   log_group_input_->stopLog();
    //   log_group_input_->setFeedbackFrequencyHz(0);
    // }

    // if (log_group_modules_) {
    //   log_group_modules_->stopLog();
    //   log_group_modules_->setFeedbackFrequencyHz(0);
    // }

    // if (log_group_input_ || log_group_modules_) {
    //   std::cout << "stopped any active logs" << std::endl;
    // }
}

} // end of namespace robot
