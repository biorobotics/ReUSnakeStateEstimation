/**
  ***************************************************************************************
  * @file    ekf.cpp
  * @author  Anoop Bhat
  * @version V1.0.0
  * @date    04-Nov-2019
  * @brief   This file is an implementation of an EKF for a snake robot
  ***************************************************************************************
  */

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "ekf.hpp"
#include "models.hpp"

using namespace Eigen;
using namespace std;

EKF::EKF(double r, double q, size_t modules) {
  num_modules = modules;

  statelen = state_length(num_modules);
  sensorlen = sensor_length(num_modules);

  MatrixXd I;
  I.setIdentity(statelen, statelen);
  R = r*I;
  S_t = I;
  I.setIdentity(sensorlen, sensorlen);
  Q = q*I;
}

EKF::EKF(MatrixXd& _R, MatrixXd& _Q, MatrixXd& _S, size_t modules) {
  num_modules = modules;

  statelen = state_length(num_modules);
  sensorlen = sensor_length(num_modules);

  R = _R;
  S_t = _S;
  Q = _Q;
}

void EKF::predict(const VectorXd& u_t, double _dt) {
  dt = _dt;

  // Compute prediction covariance using Jacobian of process model
  MatrixXd F_t(statelen, statelen);
  df(F_t, x_t, u_t, dt, num_modules);

  S_t = F_t*S_t*F_t.transpose() + R;

  // Compute state estimate
  VectorXd x_t_1(x_t);
  f(x_t, x_t_1, u_t, dt, num_modules);
}

void EKF::correct(const VectorXd& z_t) {
  // Compute Jacobian of sensor model
  MatrixXd H_t(sensorlen, statelen);
  dh(H_t, x_t, dt, num_modules);

  // Kalman gain
  MatrixXd K = S_t*H_t.transpose()*(H_t*S_t*H_t.transpose() + Q).inverse();

  // Compute expected measurement
  VectorXd h_t(sensorlen);
  h(h_t, x_t, dt, num_modules);

  // Compute difference between predicted measurement and actual
  VectorXd sensor_diff = z_t - h_t;
  for (size_t i = 0; i < sensorlen; i++) {
    if (isnan(z_t(i))) {
      sensor_diff(i) = 1000000;
    }
  }


  cout << "z_t \t " << z_t << "\n\n";
  cout << "h_t \t " << h_t << "\n\n";
  cout << "diff \t " << sensor_diff << "\n\n";

  exit(1);
  
  
  x_t = x_t + K*sensor_diff;

  MatrixXd I;
  I.setIdentity(statelen, statelen);
  S_t = (I - K*H_t)*S_t;

  // Renormalize quaternion
  Vector4d q_t;
  get_q(q_t, x_t);

  q_t = q_t/sqrt(q_t.squaredNorm());
  set_q(x_t, q_t);
}

void EKF::initialize(const VectorXd& z_t) {
  init_state(x_t, z_t, num_modules);
}