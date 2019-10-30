#pragma once

// Eigen
#include <Eigen/Dense>
// STL
#include <string>
#include <iostream>
namespace robot{

bool parse_args(int argc, char** argv, bool& visualize, bool& dummy, int& robot_type, bool& quiet);
Eigen::MatrixXd VecToSymm(Eigen::VectorXd); 
Eigen::MatrixXd VecToHomNoRot(Eigen::VectorXd);

// a helper function used in estimator to update quaternion
Eigen::MatrixXd Omega(const Eigen::VectorXd& q);
Eigen::MatrixXd skew(const Eigen::VectorXd& v);

//psedo inverse
Eigen::MatrixXd MtxPInv(Eigen::MatrixXd m);

}