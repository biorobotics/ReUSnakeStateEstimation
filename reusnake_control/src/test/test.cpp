#include <iostream>
#include <set>
#include <chrono>
#include <thread>
#include <atomic>

#include "robot/M6_leg.hpp"
#include "robot/N6_leg.hpp"
#include "parameters/robot_parameters.hpp"
#include "robot/M6_base.hpp"
#include "robot/N6_base.hpp"
#include "robot/N6_dummy.hpp"
#include "util/util.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_control");

  // std::unique_ptr<robot::RobotBase> robot_base;
  // robot_base = robot::N6Base::create();
  /* test!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 2019 03 31 */
  // robot::M6LeftLeg my_test_left_leg(0);
  // robot::M6RightLeg my_test_right_leg(1);
  // Eigen::VectorXd curr_pos_ang(3);curr_pos_ang << 10/180.0*3.14,-10/180.0*3.14,-90/180.0*3.14;
  // Eigen::VectorXd curr_vel_angs(3); curr_vel_angs << 0,0,0;
  // Eigen::VectorXd curr_tau_nm(3); curr_tau_nm<< 0,0,0;
  
  // my_test_left_leg.updateState(curr_pos_ang, curr_vel_angs, curr_tau_nm);
  // Eigen::VectorXd test_FK(3);
  // Eigen::VectorXd angles(3); angles << 0,0,0; angles = curr_pos_ang;
  // Eigen::VectorXd desired_pos(3);

  //   test_FK = my_test_left_leg.computeFK(angles);
  //   std::cout << "Left Computed FK " << test_FK.transpose() << std::endl;
  // for (int i = 0;i < 400; i++) {

  //   desired_pos  = test_FK;
  //   my_test_left_leg.computeIK(angles, desired_pos);

  //   std::cout << "Left Computed IK " << angles.transpose() << std::endl;

  //   // my_test_left_leg.updateState(angles, curr_vel_angs, curr_tau_nm);
  //   test_FK = my_test_left_leg.computeFK(angles);
  //   std::cout << "Left Computed FK again " << test_FK.transpose() << std::endl;
  // }

  // Eigen::MatrixXd Jb = my_test_left_leg.computerJBody();

  // std::cout << "Left Computed Jacobian body" << std::endl << Jb << std::endl;


  // my_test_right_leg.updateState(curr_pos_ang, curr_vel_angs, curr_tau_nm);
  // test_FK = my_test_right_leg.computeFK();
  // std::cout << "Right Computed FK " << test_FK.transpose() << std::endl;

  // desired_pos  = test_FK;
  // my_test_right_leg.computeIK(angles, desired_pos);

  // std::cout << "Right Computed IK " << angles.transpose() << std::endl;

  // my_test_right_leg.updateState(angles, curr_vel_angs, curr_tau_nm);
  // test_FK = my_test_right_leg.computeFK();
  // std::cout << "Right Computed FK again " << test_FK.transpose() << std::endl;

  // Jb = my_test_right_leg.computerJBody();

  // std::cout << "Right Computed Jacobian body" << std::endl << Jb << std::endl;

  // // robot::RobotParameters robot_params;
  // // std::unique_ptr<robot::M6Base> m6Robot = robot::M6Base::create();
  

  // // test VecToSymm for Inertia matrix
  // Eigen::VectorXd I_tilde(6);
  // I_tilde << 1,2,3,4,5,6;
  // std::cout << "Given I_tilde = \n" << I_tilde.transpose() << "\n, get I = \n" << robot::VecToSymm(I_tilde) << std::endl;

  ////////////////////////////////////////////////
  // test for FK IK in robot base
  ////////////////////////////////////////////////
  std::unique_ptr<robot::RobotBase> robot_base;
  robot_base = robot::N6Dummy::create();
  Eigen::VectorXd test_angles(3);  test_angles << 0, 0, -0.1;//test_angles << -0.09,-0.54,1.81;
  Eigen::VectorXd computed_angles(3);
  Eigen::VectorXd computed_pos = robot_base->getLeg(4)->computeFK(test_angles);

  robot_base->getLeg(4)->computeIK(computed_angles, computed_pos); // in s
  // robot_base->computeIK(4, computed_angles, computed_pos); // in c
  std::cout << test_angles.transpose() << computed_pos.transpose() << computed_angles.transpose() << std::endl;

  assert((computed_angles-test_angles).norm()<1e-3);

  /////////////////////////////////////////////////
  return 0;
}