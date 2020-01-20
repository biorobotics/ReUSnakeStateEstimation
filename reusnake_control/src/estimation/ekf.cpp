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

EKF::EKF(double q, double r, size_t modules) {
  num_modules = modules;

  statelen = state_length(num_modules);
  sensorlen = sensor_length(num_modules);

  MatrixXd I;
  I.setIdentity(statelen, statelen);
  Q = q*I;
  S_t = I;
  I.setIdentity(sensorlen, sensorlen);
  R = r*I;

  h_t = VectorXd::Zero(sensorlen);
}

EKF::EKF(MatrixXd& _Q, MatrixXd& _R, MatrixXd& _S, size_t modules) {
  num_modules = modules;

  statelen = state_length(num_modules);
  sensorlen = sensor_length(num_modules);

  Q = _Q;
  S_t = _S;
  R = _R;
}

void EKF::predict(const VectorXd& u_t, double _dt) {
  dt = _dt;

  // Compute prediction covariance using Jacobian of process model
  MatrixXd F_t(statelen, statelen);
  df(F_t, x_t, u_t, dt, num_modules);

  S_t = F_t*S_t*F_t.transpose() + Q;

  // Compute state estimate
  VectorXd x_t_1(x_t);
  f(x_t, x_t_1, u_t, dt, num_modules);
}

void EKF::correct(const VectorXd& z_t) {
  // Compute Jacobian of sensor model
  MatrixXd H_t(sensorlen, statelen);
  dh(H_t, x_t, dt, num_modules);
  
  for (size_t i = 0; i < sensorlen; i++) {
    if (isnan(z_t(i))) {
      R(i, i) = 1000000;
      z_t(i) = 0;
    }
  }

  // For computing Kalman gain
  MatrixXd tmp = H_t*S_t*H_t.transpose() + R;

  // Compute expected measurement
  //VectorXd h_t(sensorlen);
  h(h_t, x_t, dt, num_modules);

  // Compute difference between predicted measurement and actual
  VectorXd sensor_diff = z_t - h_t;

  x_t = x_t + S_t*H_t.transpose()*(tmp.colPivHouseholderQr().solve(sensor_diff));

  MatrixXd I;
  I.setIdentity(statelen, statelen);
  MatrixXd K = S_t*H_t.transpose()*tmp.colPivHouseholderQr().inverse();
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
