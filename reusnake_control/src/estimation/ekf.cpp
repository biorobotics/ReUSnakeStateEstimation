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

using namespace Eigen;
using namespace std;

static const double epsilon = 0.001;

EKF::EKF(double q, double r,
         void (*_f_func)(VectorXd& x_t, const VectorXd& x_t_1,
                         const VectorXd& u_t, double dt,
                         size_t num_modules),
         void (*_h_func)(VectorXd& z_t, const VectorXd& x_t, double dt,
                         size_t num_modules),
         size_t (*_state_length)(size_t num_modules),
         size_t (*_sensor_length)(size_t num_modules),
         void (*_init_state)(VectorXd& x_t, const VectorXd& z_t, size_t num_modules),
         size_t modules) {
  num_modules = modules;

  state_length = _state_length(num_modules);
  sensor_length = _sensor_length(num_modules);

  MatrixXd I;
  I.setIdentity(state_length, state_length);
  R = r*I;
  S_t = 0.1*I;
  I.setIdentity(sensor_length, sensor_length);
  Q = q*I;
  f_func = _f_func;
  h_func = _h_func;
  init_state = _init_state;
}

void EKF::predict(const VectorXd& u_t, double _dt) {
  dt = _dt;

  // Compute prediction covariance using Jacobian of process model
  MatrixXd F_t(state_length, state_length);
  df(F_t, x_t, u_t);
  
  S_t = F_t*S_t*F_t.transpose() + R;

  // Compute state estimate
  VectorXd x_t_1(state_length);
  for (size_t i = 0; i < state_length; i++) {
    x_t_1(i) = x_t(i);
  }
  f_func(x_t, x_t_1, u_t, dt, num_modules);
}

void EKF::correct(const VectorXd& z_t) {
  // Compute Jacobian of sensor model
  MatrixXd H_t(sensor_length, state_length);
  dh(H_t, x_t);

  // Kalman gain
  MatrixXd K = S_t*H_t.transpose()*(H_t*S_t*H_t.transpose() + Q).inverse();

  // Compute expected measurement
  VectorXd h_t(sensor_length);
  h_func(h_t, x_t, dt, num_modules);

  // Compute difference between predicted measurement and actual
  VectorXd sensor_diff = z_t - h_t;
  for (size_t i = 0; i < sensor_length; i++) {
    if (isnan(z_t(i))) {
      sensor_diff(i) = 1000000;
    }
  }

  x_t = x_t + K*sensor_diff;
  MatrixXd I;
  I.setIdentity(state_length, state_length);
  S_t = (I - K*H_t)*S_t;
}

void EKF::initialize(const VectorXd& z_t) {
  init_state(x_t, z_t, num_modules);
}

void EKF::df(MatrixXd& F_t, VectorXd x_t_1, const VectorXd& u_t) {
  for (size_t col = 0; col < state_length; col++) {
    // Perturb state variable corresponding to this column of J
    x_t_1(col) += epsilon;
    VectorXd x_t_plus(state_length);
    f_func(x_t_plus, x_t_1, u_t, dt, num_modules);
    x_t_1(col) -= 2*epsilon;
    VectorXd x_t_minus(state_length);
    f_func(x_t_minus, x_t_1, u_t, dt, num_modules);
    x_t_1(col) += epsilon;

    // Compute derivative
    VectorXd dx_t = (x_t_plus - x_t_minus)/(2*epsilon);

    // Populate J
    for (size_t row = 0; row < state_length; row++) {
      F_t(row, col) = dx_t(row);
    }
  }
}

void EKF::dh(MatrixXd& H_t, VectorXd x_t) {
  for (size_t col = 0; col < state_length; col++) {
    // Perturb state variable corresponding to this column of J
    x_t(col) += epsilon;
    VectorXd z_t_plus(sensor_length);
    h_func(z_t_plus, x_t, dt, num_modules);
    x_t(col) -= 2*epsilon;
    VectorXd z_t_minus(sensor_length);
    h_func(z_t_minus, x_t, dt, num_modules);
    x_t(col) += epsilon;

    // Compute derivative
    VectorXd dz_t = (z_t_plus - z_t_minus)/(2*epsilon);

    // Populate J
    for (size_t row = 0; row < sensor_length; row++) {
      H_t(row, col) = dz_t(row);
    }
  }
}
