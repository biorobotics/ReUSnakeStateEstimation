#include <Eigen/Dense>
#include <unistd.h>
#include <cmath>
#include "models.hpp"
#include <iostream>
#include "ekf.hpp"

using namespace Eigen;
using namespace std;

static const double dt = 0.1;
static const size_t num_modules = 2;
static const double g = 9.8;

// Prints orientation (angle/axis)
void print_orientation(VectorXd& x_t) {
  Vector4d q_t;
  get_q(q_t, x_t);
  double angle = 2*acos(q_t(0));
  double s_angle = sin(angle/2);
  if (s_angle == 0) {
    printf("angle: %lf, axis: 0 1 0 0\n", angle);
  } else {
    Vector3d axis(q_t(1)/s_angle,
                  q_t(2)/s_angle,
                  q_t(3)/s_angle);
    printf("angle: %lf, axis: %lf %lf %lf\n", angle, axis(0), axis(1), axis(2));
  }
}

// Print angular velocity
void print_angvel(VectorXd& x_t) {
  Vector3d w_t;
  get_w(w_t, x_t);
  printf("Angular velocity: %lf %lf %lf\n\n", w_t(0), w_t(1), w_t(2));
}

// test sensor model with 2 modules
int main() {
  VectorXd z_t(sensor_length(num_modules));

  // Module 1 initially has its x-axis pointing down
  Vector3d alpha_1(-g, 0, 0);
  // Module 2 initially has its y-axis pointing down
  // (pi/2 rotation about module 1 z-axis)
  Vector3d alpha_2(0, -g, 0);
  set_alpha(z_t, alpha_1, 0, num_modules);
  set_alpha(z_t, alpha_2, 1, num_modules);

  // Virtual chassis is spinning about the -y-axis
  // Module 1 is spinning about the x-axis
  Vector3d gamma_1(1, 0, 0);
  // Module 2 is spinning about the y-axis
  Vector3d gamma_2(0, 1, 0);
  set_gamma(z_t, gamma_1, 0, num_modules);
  set_gamma(z_t, gamma_2, 1, num_modules);

  EKF ekf(1, 1, &f, &h, &state_length, &sensor_length, &init_state,
          num_modules);

  ekf.initialize(z_t);

  // Virtual chassis is spinning about the -y-axis
  Vector3d w_t(0, -1, 0);
  set_w(ekf.x_t, w_t);

  printf("Starting orientation:\n");
  print_orientation(ekf.x_t);

  // All joints are currently zero, and we're not commanding them otherwise
  VectorXd u_t(num_modules);
  u_t.setZero();

  printf("Starting filter...\n");
  for (size_t i = 0; i < 50; i++) {
    ekf.predict(u_t, dt);
    ekf.correct(z_t);

    print_orientation(ekf.x_t);
    print_angvel(ekf.x_t);
  }
}
