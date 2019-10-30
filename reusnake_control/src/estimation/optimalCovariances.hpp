
#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>
#include "estimator.hpp"
#include "ukf.hpp"


using namespace Eigen;
using namespace std;
 
float computeCost(const VectorXd x_ukf, const VectorXd x_gt);

float gradFunction(robot::Estimator* estimator, const VectorXd x_gt, const Vector3d& acc_b, const Vector3d& gyro_b, 
    const std::vector<Eigen::VectorXd> ee_pos_c_list, const std::vector<Eigen::VectorXd> ee_vel_c_list, const std::vector<int> contact_list,
    const double dt, const float phase, const float covarianceScale);

float getOptimalScale(robot::Estimator* estimator, int num_iters, const VectorXd x_gt, const Vector3d& acc_b, const Vector3d& gyro_b, 
    const std::vector<Eigen::VectorXd> ee_pos_c_list, const std::vector<Eigen::VectorXd> ee_vel_c_list, const std::vector<int> contact_list,
    const double dt, const float phase, const float covarianceScale);
