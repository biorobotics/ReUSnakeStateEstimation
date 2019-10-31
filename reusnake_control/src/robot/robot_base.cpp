#include "robot_base.hpp"

namespace robot {

RobotBase::RobotBase()
{
  last_fbk_time_ = std::chrono::steady_clock::now();
  curr_fbk_time_ = std::chrono::steady_clock::now();
   // pose publisher
   // TODO: give it different name according to different derived classes

  return;
}

RobotBase::~RobotBase(){}

bool RobotBase::init() {
  /* load robot related parameters according to robot type 
  /  This part will be moved to robot init function  
  */
  return true;
}

void RobotBase::gait_sidewinding(double t) {
  // sidewinding parameters
  double wS = 0.5;
  double wT= 4;
  double A_even = 0.6;
  double A_odd = .8;
  double delta = M_PI/4; // right
  // delta = -pi/4; // left
  double beta_odd = 0;
  double beta_even = 0;
  Eigen::VectorXd amp_mult = Eigen::VectorXd::Ones(numModules);

  Eigen::VectorXd angles(numModules);
  Eigen::VectorXd vels(numModules); vels *= NAN;
  Eigen::VectorXd torques(numModules); torques*= NAN;
  // torques(0) = NAN; vels(0) = NAN;
  for (int i = 0; i < numModules; ++i)
  {
    if (i % 2 == 0) {// it is even
        angles(i) = beta_even + A_even*amp_mult(i)*sin(wS*i-wT*t + delta);
    } else {
        angles(i) = beta_odd + A_odd*amp_mult(i)*sin(wS*i-wT*t);
    }
  }

  setCommand( &angles, &vels, &torques);

  // std::cout << "cmd angles: ";
  // for (int i = 0; i < numModules; ++i) {
  //   std::cout << angles(i) << "\t";
  // }
  // std::cout << std::endl;
}


} // namespace robot