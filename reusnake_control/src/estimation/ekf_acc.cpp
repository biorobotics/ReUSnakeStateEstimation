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
#include "ekf_acc.hpp"
#include "models_acc.hpp"
#include <numeric> // std::iota
#include <algorithm> // std::sort, std::stable_sort

using namespace Eigen;
using namespace std;

vector<size_t> sort_indices(const vector<double> &v) {
  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indices based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values 
  stable_sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

EKF::EKF(double q, double r, size_t modules) {
  num_modules = modules;

  statelen = state_length(num_modules);
  sensorlen = sensor_length(num_modules);

  MatrixXd I;
  I.setIdentity(3, 3);
  Q = q*I;
  S_t = I;
  I.setIdentity(sensorlen, sensorlen);
  R = r*I;

  x_t = VectorXd::Zero(statelen);
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
  MatrixXd F_t(3, 3);
  df(F_t, x_t, u_t, dt, num_modules);

  S_t = F_t*S_t*F_t.transpose() + Q;

  // Compute state estimate
  VectorXd x_t_1(x_t);
  f(x_t, x_t_1, u_t, dt, num_modules);
}

void EKF::correct(const VectorXd& z_t, vector<double> angles) {
  // Compute Jacobian of sensor model
  MatrixXd H_t(sensorlen, 3);
  dh(H_t, x_t, dt, num_modules, angles);

  // Compute expected measurement
  h(h_t, x_t, dt, num_modules, angles);

  // Compute difference between predicted measurement and actual
  VectorXd sensor_diff = z_t - h_t;
  
  MatrixXd old_R = R.block(0, 0, sensorlen, sensorlen);
  for (size_t i = 0; i < sensorlen; i++) {
    if (isnan(z_t(i))) {
      R(i, i) = 1000000;
      sensor_diff(i) = -h_t(i); // i.e. treat z_t(i) as 0
    }
  }

  // Temporary matrix that we'll use a few times in the future
  MatrixXd SHT = S_t*H_t.transpose();
  // Innovation covariance, for computing Kalman gain
  MatrixXd inn_cov = H_t*SHT + R;

  // Outlier detection and removal 
  LDLT<MatrixXd> inn_cov_ldlt = inn_cov.ldlt();

  // Vector whose ith element is the Mahalanobis distance when ith sensor is eliminated
  vector<double> ds(num_modules); 
  for (size_t s = 0; s < num_modules; s++) {
    size_t mblock_size = s*3;
    size_t nblock_start = s*3 + 3;
    size_t nblock_size = sensorlen - mblock_size - 3;
    
    // Matrix that moves ith sensor from residual to the end
    MatrixXd G_s = MatrixXd::Zero(sensorlen, sensorlen);
    G_s.block(0, 0, mblock_size, mblock_size) = MatrixXd::Identity(mblock_size, mblock_size);
    G_s.block(mblock_size, nblock_start, nblock_size, nblock_size) = MatrixXd::Identity(nblock_size, nblock_size);
    G_s.block(sensorlen - 3, mblock_size, 3, 3) = Matrix3d::Identity();

    VectorXd sensor_diff_elim(sensorlen - 3); // sensor difference with ith sensor eliminated
    sensor_diff_elim.head(mblock_size) = sensor_diff.head(mblock_size);
    sensor_diff_elim.tail(nblock_size) = sensor_diff.tail(nblock_size);

    MatrixXd S_prime_inv = G_s*inn_cov_ldlt.solve(G_s.transpose());

    MatrixXd A = S_prime_inv.block(0, 0, sensorlen - 3, sensorlen - 3);
    MatrixXd B = S_prime_inv.block(0, sensorlen - 3, sensorlen - 3, 3);
    MatrixXd C = S_prime_inv.block(sensorlen - 3, sensorlen - 3, 3, 3);

    // Inverse of the innovation covariance when the ith sensor is eliminated
    MatrixXd inn_cov_elim_inv = A - B*C.inverse()*B.transpose();

    ds[s] = sensor_diff_elim.transpose()*inn_cov_elim_inv*sensor_diff_elim;
  }

  // Compute mean and variance of Mahalanobis distances corresponding to inliers
  // (outliers are the four lowest ones)
  double mean_d = 0;
  double var_d = 0;
  vector<size_t> indices = sort_indices(ds);
  for (size_t i = 4; i < num_modules; i++) {
    double d = ds[indices[i]];
    mean_d += d;
    var_d += d*d;
  }
  var_d -= mean_d*mean_d/(num_modules - 4);
  mean_d /= (num_modules - 4);
  var_d /= (num_modules - 5);
  
  // Mark outliers based on Mahalanobis distance of their Mahalanobis distance
  // from the mean Mahalanobis distance of the inliers
  for (size_t i = 0; i < num_modules; i++) {
    size_t s = indices[i];
    double w = pow(ds[s] - mean_d, 2)/var_d;
    if (w > 15 || s == 6) {
      size_t j = 3*s;
      R.block(j, j, 3, 3) = 1000000*Matrix3d::Identity();
    }
  }

  // Compute new innovation covariance
  inn_cov = H_t*SHT + R;

  inn_cov_ldlt = inn_cov.ldlt();
  //x_t = x_t + SHT*(inn_cov_ldlt.solve(sensor_diff));
  Quaterniond error_q;
  error_q.vec() = SHT*(inn_cov_ldlt.solve(sensor_diff))/2;
  error_q.w() = sqrt(1 - error_q.vec().squaredNorm());
  Quaterniond q_t(x_t(0), x_t(1), x_t(2), x_t(3));
  q_t = q_t*error_q;
  x_t(0) = q_t.w();
  x_t.tail(3) = q_t.vec();
  x_t = x_t/x_t.norm();

  MatrixXd I = MatrixXd::Identity(3, 3);

  // SHT*inn_cov_ldlt.inverse() is the Kalman gain
  S_t = (I - SHT*inn_cov_ldlt.solve(H_t))*S_t;

  /*
  // Renormalize quaternion
  Vector4d q_t;
  get_q(q_t, x_t);

  q_t = q_t/sqrt(q_t.squaredNorm());
  set_q(x_t, q_t);
  */
  R = old_R;
}

void EKF::initialize(const VectorXd& z_t) {
  init_state(x_t, z_t, num_modules);
}
