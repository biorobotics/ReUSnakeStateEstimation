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

void EKF::predict(const VectorXd& u_t, double _dt) {
  dt = _dt;

  // Jacobian of process model
  MatrixXd F_t(statelen, statelen);

  // Previous state
  VectorXd x_t_1(x_t);
  
  f(x_t, x_t_1, u_t, dt, num_modules, body_frame_module);
  df(F_t, x_t_1, u_t, dt, num_modules, body_frame_module);

  S_t = F_t*S_t*F_t.transpose() + Q;
}

void EKF::correct(const VectorXd& z_t) {
  MatrixXd H_t(sensorlen, statelen); // Jacobian for sensor model

  dh(H_t, x_t, dt, num_modules, body_frame_module, vc);
  vc = h(h_t, x_t, dt, num_modules, body_frame_module, vc);

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

  // Outlier detection and removal (we don't perform on encoders, so remove
  // them temporarily from residual)
  size_t sensorlen_short = sensorlen - num_modules;
  size_t sensornum = 2*num_modules; // one gyro and one accelerometer per module
  VectorXd sensor_diff_short = sensor_diff.tail(sensorlen_short);

  LDLT<MatrixXd> inn_cov_ldlt = inn_cov.block(num_modules, num_modules, sensorlen_short, sensorlen_short).ldlt();

  // Vector whose ith element is the Mahalanobis distance when ith sensor is eliminated
  vector<double> ds(sensornum); 
  for (size_t s = 0; s < sensornum; s++) {
    size_t mblock_size = s*3;
    size_t nblock_start = s*3 + 3;
    size_t nblock_size = sensorlen_short - mblock_size - 3;
    
    // Matrix that moves ith sensor from residual to the end
    MatrixXd G_s = MatrixXd::Zero(sensorlen_short, sensorlen_short);
    G_s.block(0, 0, mblock_size, mblock_size) = MatrixXd::Identity(mblock_size, mblock_size);
    G_s.block(mblock_size, nblock_start, nblock_size, nblock_size) = MatrixXd::Identity(nblock_size, nblock_size);
    G_s.block(sensorlen_short - 3, mblock_size, 3, 3) = Matrix3d::Identity();

    VectorXd sensor_diff_elim(sensorlen_short - 3); // sensor difference with ith sensor eliminated
    sensor_diff_elim.head(mblock_size) = sensor_diff_short.head(mblock_size);
    sensor_diff_elim.tail(nblock_size) = sensor_diff_short.tail(nblock_size);

    MatrixXd S_prime_inv = G_s*inn_cov_ldlt.solve(G_s.transpose());

    MatrixXd A = S_prime_inv.block(0, 0, sensorlen_short - 3, sensorlen_short - 3);
    MatrixXd B = S_prime_inv.block(0, sensorlen_short - 3, sensorlen_short - 3, 3);
    MatrixXd C = S_prime_inv.block(sensorlen_short - 3, sensorlen_short - 3, 3, 3);

    // Inverse of the innovation covariance when the ith sensor is eliminated
    MatrixXd inn_cov_elim_inv = A - B*C.inverse()*B.transpose();

    ds[s] = sensor_diff_elim.transpose()*inn_cov_elim_inv*sensor_diff_elim;
  }

  // Compute mean and variance of Mahalanobis distances corresponding to inliers
  // (outliers are the four lowest ones)
  double mean_d = 0;
  double var_d = 0;
  vector<size_t> indices = sort_indices(ds);
  for (size_t i = 4; i < sensornum; i++) {
    double d = ds[indices[i]];
    mean_d += d;
    var_d += d*d;
  }
  var_d -= mean_d*mean_d/(sensornum - 4);
  mean_d /= (sensornum - 4);
  var_d /= (sensornum - 5);
  
  // Mark outliers based on Mahalanobis distance of their Mahalanobis distance
  // from the mean Mahalanobis distance of the inliers
  for (size_t i = 0; i < sensornum; i++) {
    size_t s = indices[i];
    double w = pow(ds[s] - mean_d, 2)/var_d;
    if (w > 15) {
      size_t j = num_modules + 3*s;
      R.block(j, j, 3, 3) = 1000000*Matrix3d::Identity();
    }
  }

  // Compute new innovation covariance
  inn_cov = H_t*SHT + R;

  inn_cov_ldlt = inn_cov.ldlt();
  x_t = x_t + SHT*(inn_cov_ldlt.solve(sensor_diff));

  MatrixXd I = MatrixXd::Identity(statelen, statelen);

  // SHT*inn_cov_ldlt.inverse() is the Kalman gain
  S_t = (I - SHT*inn_cov_ldlt.solve(H_t))*S_t;

  // Renormalize quaternion
  Vector4d q_t;

  get_q(q_t, x_t);

  q_t = q_t/sqrt(q_t.squaredNorm());
  set_q(x_t, q_t);

  R = old_R;
}

void EKF::initialize(size_t modules, short _body_frame_module, const VectorXd& z_t) {
  num_modules = modules;
  body_frame_module = _body_frame_module;

  statelen = state_length(num_modules);
  sensorlen = sensor_length(num_modules);

  x_t = VectorXd::Zero(statelen);
  h_t = VectorXd::Zero(sensorlen);

  vc = init_state(x_t, z_t, num_modules, body_frame_module);
}
