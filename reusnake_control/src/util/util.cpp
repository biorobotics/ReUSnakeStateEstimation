#include "util.hpp"

namespace robot{

bool parse_args(int argc, char** argv, bool& visualize, bool& dummy, int& robot_type, bool& quiet)
{
  visualize = false;
  dummy = false;
  quiet = false;
  bool valid = true;
  robot_type = -1;
  int idx = 1; // Ignore program name!
  for (;idx < argc; ++idx) {
    std::string str_arg(argv[idx]);
    if (str_arg == "-h") {
      std::cout << "Hexapod control usage:\n" <<
      "    -d\n" <<
      "        Dummy hexapod; no modules are connected to.\n\n" <<
      "    -v\n" <<
      "        Visualize -- show a simple rendering of the robot.\n\n" <<
      "    -t <type>\n" <<
      "        The type of robot. This is very important. \n" << 
      "        Type = 0 The robot is Mat6 (AKA Hebi Daisy) \n" <<
      "        Type = 1 The robot is N=6 (The first prototype of Titan6 robot) \n\n" <<
      "        Type = 2 The robot is Dummy Mat6 \n\n" <<
      "    -q\n" <<
      "        Quiet mode (no dialog messages; waits and tries to continue on failure such as no modules on the network).\n\n" <<
      "    -h\n" <<
      "        Print this help and return." << std::endl;
      return false;
    }
    else if (str_arg == "-v") {
      visualize = true;
      continue;
    }
    else if (str_arg == "-t" && idx + 1 < argc) {
      ++idx;
      try {
        robot_type = std::stoi(std::string(argv[idx]));
      } 
      catch (std::invalid_argument) {
        std::cout << "You must provide a correct type of robot you are using! Use \"-h\" for list of robots." << std::endl;
      }
      continue;
    }
    else if (str_arg == "-d") {
      dummy = true;
      continue;
    }
    else if (str_arg == "-q") {
      quiet = true;
      continue;
    }
    else {
      valid = false;
      break;
    }
  }
  // Do all exclusive argument checks here
  if (robot_type == -1) {
    std::cout << "You must confirm the type of robot you are using! Add \"-t <type>\" as argument." << std::endl;
    return false;
  }
  if (!valid) {
    std::cout << "Invalid combination of arguments! Use \"-h\" for usage." << std::endl;
    return false;
  }
  return true;
}

Eigen::MatrixXd VecToSymm(Eigen::VectorXd I_tilde) {
  assert(I_tilde.size() == 6);
  Eigen::MatrixXd I(3,3);
  I <<  I_tilde(0), I_tilde(1), I_tilde(2),
        I_tilde(1), I_tilde(3), I_tilde(4),
        I_tilde(2), I_tilde(4), I_tilde(5);
  return I;
}

Eigen::MatrixXd VecToHomNoRot(Eigen::VectorXd position) {
  assert(position.size() == 3);
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(4,4);
  H.topRightCorner<3,1>() = position;
  return H;
}


// v order follows  q.coeffs (x y z w)
Eigen::MatrixXd Omega(const Eigen::VectorXd& q)
{
    double x = q(0);
    double y = q(1);
    double z = q(2);
    double w = q(3);

    Eigen::MatrixXd m(4,4);
    m <<     w,  -z,   y,  x,
             z,   w,  -x,  y,
            -y,   x,   w,  z,
            -x,  -y,  -z,  w;
    return m;
}

Eigen::MatrixXd skew(const Eigen::VectorXd& v)
{
    double x = v(0);
    double y = v(1);
    double z = v(2);

    Eigen::MatrixXd m(3,3);
    m <<     0,  -z,   y,  
             z,   0,  -x, 
            -y,   x,   0;
    return m;
}


Eigen::MatrixXd MtxPInv(Eigen::MatrixXd m) 
{
  auto svd = m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto & singularValues = svd.singularValues();

  Eigen::MatrixXd singularValuesInv = Eigen::MatrixXd(m.cols(), m.rows());
  singularValuesInv.setZero();
  double pInvtoler = 1e-6;
  for (int i = 0; i < singularValues.size(); i++) {
    if (singularValues(i) > pInvtoler) {
      singularValuesInv(i,i) = 1.0f / singularValues(i);
    } else {
      singularValuesInv(i,i) = 0.0f;
    }
  }
  Eigen::MatrixXd pinvMtx = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
  return pinvMtx;
}

} // namespace robot