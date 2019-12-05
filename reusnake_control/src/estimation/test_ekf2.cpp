#include <Eigen/Dense>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include "ekf.hpp"
#include "models.hpp"

using namespace Eigen;
using namespace std;

static const double dt = 0.1;
static const size_t num_modules = 2;
static const double g = 9.8;

// Prints head orientation zyz Euler angles
void print_orientation(VectorXd& x_t) {
  Vector4d qvec;
  get_head(qvec, x_t, num_modules);
  Quaterniond q_t(qvec(0), qvec(1), qvec(2), qvec(3));
  Vector3d euler = q_t.toRotationMatrix().eulerAngles(2, 1, 2);
  printf("head zyz euler angles: %lf %lf %lf\n", euler(0), euler(1), euler(2));
}

// Print angular velocity
void print_angvel(VectorXd& x_t) {
  Vector3d w_t;
  get_w(w_t, x_t);
  printf("Angular velocity: %lf %lf %lf\n\n", w_t(0), w_t(1), w_t(2));
}

int main() {
  size_t statelen = state_length(num_modules);
  size_t sensorlen = sensor_length(num_modules);
  VectorXd z_t(sensorlen);

  Vector3d alpha(0, g, 0);
  for (size_t i = 0; i < num_modules; i++) {
    set_alpha(z_t, alpha, i, num_modules);
  }

  // Virtual chassis is spinning about the x-axis
  Vector3d gamma(0, 0, 1);
  for (size_t i = 0; i < num_modules; i++) {
    set_gamma(z_t, gamma, i, num_modules);
  }

  MatrixXd R = MatrixXd::Identity(statelen, statelen);
  MatrixXd Q = 0.05*MatrixXd::Identity(sensorlen, statelen);
  MatrixXd S = 0.01*MatrixXd::Identity(statelen, statelen);
  S.block(7, 7, 3, 3) = 100*Matrix3d::Identity();
  EKF ekf(R, Q, S, num_modules);

  ekf.initialize(z_t);

  // Virtual chassis is spinning about the x-axis
  Vector3d w_t(1, 0, 0);
  set_w(ekf.x_t, w_t);

  printf("Starting orientation:\n");
  print_orientation(ekf.x_t);

  // All joints are currently zero, and we're not commanding them otherwise
  VectorXd u_t(num_modules);
  u_t.setZero();

  printf("Starting filter...\n");
  for (size_t i = 0; i < 20; i++) {
    ekf.predict(u_t, dt);
    
    double t = i*dt;
    
    alpha(0) = g*sin(t);
    alpha(1) = g*cos(t);
    alpha(2) = 0;
    for (size_t j = 0; j < num_modules; j++) {
      set_alpha(z_t, alpha, j, num_modules);
    }

    ekf.correct(z_t);
    print_orientation(ekf.x_t);
    print_angvel(ekf.x_t);
  }
}
