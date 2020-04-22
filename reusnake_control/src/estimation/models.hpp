/**
  ******************************************************************************
  * @file    models.hpp
  * @author  Anoop Bhat
  * @date    02-Nov-2019
  * @version V1.0.0
  * @brief   This file contains the functions for the process model and
  *          measurement model for a snake Kalman filter.
  ******************************************************************************
  */
#pragma once

#include <Eigen/Dense>

using namespace Eigen;

/*
 * get_head_kinematics: gets the expected IMU measurements for an IMU
 * at the snake's head
 * ARGUMENTS
 * accel: expected accelerometer measurement
 * ang_vel: expected gyro measurement
 * x_t: current state
 * num_modules: number of modules in snake
 * dt: time interval used for differentiation
 * body_frame_module: module to use as body frame. If -1, use virtual chassis
 */
void get_head_kinematics(Vector3d& accel, Vector3d& ang_vel, const VectorXd& x_t,
                         size_t num_modules, double dt,
                         short body_frame_module, const Matrix4d& prev_vc);

Vector3d get_body_displacement(const vector<double>& angles, const vector<double>& prev_angles,
                               const Matrix3d& R_vc_world, 
                               size_t num_modules, const Matrix4d& prev_vc);

/*
 * f: predicts current state given previous state and time interval
 * (process model)
 * ARGUMENTS
 * x_t: current state, returned with the following convention
 *      - a_t: body frame acceleration (3-vector)
 *      - q_t: body frame orientation (quaternion [qw, qx, qy, qz])
 *      - w_t: body frame angular velocity (3-vector)
 *      - theta_t: joint angles (vector with length num_modules)
 *      - theta_dot_t: joint velocities (vector with length num_modules)
 * x_t_1: previous state, passed with above convention
 * u_t: control signal (in this case a vector of length num_modules + 3
        containing commanded joint angles and gyro measurement at body frame,
        or in the case of virtual chassis, the same but without the gyro
        measurement)
 * dt: time interval between x_t and x_t_1
 * num_modules: number of modules in snake
 * body_frame_module: module to use as body frame. If -1, use virtual chassis
 */
void f(VectorXd& x_t, const VectorXd& x_t_1, const VectorXd& u_t,
       double dt, size_t num_modules, short body_frame_module,
       const Matrix4d& prev_vc, const vector<double>& angles);

/*
 * h: predicts sensor measurements given current state (measurement model)
 * ARGUMENTS
 * z_t: predicted sensor measurements, returned with the following convention
 *      - phi_t: joint angles (vector with length num_modules)
        - alpha_t: accelerometer values (vector with length 3*num_modules)
        - gamma_t: gyro values (vector with length 3*num_modules)
 * x_t: current state, passed with same conventions as f
 * dt: time interval between x_t and x_t_1
 * num_modules: number of modules in snake 
 * body_frame_module: module to use as body frame. If -1, use virtual chassis
 * prev_vc: the previous virtual chassis, for correction
 * RETURN: if body_frame_module is -1, return the current virtual chassis,
 * consistent with the previous one. Otherwise, undefined behavior
 */
Matrix4d h(VectorXd& z_t, const VectorXd& x_t, double dt, size_t num_modules,
       short body_frame_module, const Matrix4d& prev_vc, const vector<double>& angles);

/*
* df: computes Jacobian of f
* F_t: Jacobian of f
* x_t_1: state to evaluate Jacobian
* u_t: control signal
* dt: time step
* num_modules: number of modules in the snake
* body_frame_module: module to use as body frame. If -1, use virtual chassis
*/
void df(MatrixXd& F_t, const VectorXd& x_t_1, const VectorXd& u_t, double dt,
        size_t num_modules, short body_frame_module, Matrix4d& prev_vc,
        const vector<double>& angles);

/*
* dh: computes Jacobian of h
* H_t: Jacobian of h
* x_t: state to evaluate Jacobian
* dt: time step
* num_modules: number of modules in the snake
* body_frame_module: module to use as body frame. If -1, use virtual chassis
* prev_vc: the previous virtual chassis, for correction
*/
void dh(MatrixXd& H_t, const VectorXd& x_t, double dt, size_t num_modules,
        short body_frame_module, const Matrix4d& prev_vc,
        const vector<double>& angles);

/*
 * state_length: computes the length of the state vector assumed by this
 *               model
 * ARGUMENTS
 * num_modules: number of modules in snake
 * RETURN
 * length of state vector that this model uses
 */
inline size_t state_length(size_t num_modules) {
  return 13 + num_modules;
}

/*
 * sensor_length: computes the length of the sensor vector assumed by this
 *              model
 * ARGUMENTS
 * num_modules: number of modules in snake
 * RETURN
 * length of sensor vector that this model uses
 */
inline size_t sensor_length(size_t num_modules) {
  return 6*num_modules;
}

/*
 * init_state: initializes the state vector using sensor readings
 * ARGUMENTS
 * x_t: state vector to initialize
 * z_t: measurements for use in initialization
 * num_modules: number of modules in snake
 * body_frame_module: module to use as body frame. If -1, use virtual chassis
 * RETURN
 * virtual chassis if body_frame_module == -1, undefined otherwise
 */
Matrix4d init_state(VectorXd& x_t, const VectorXd& z_t, size_t num_modules,
                    short body_frame_module, const vector<double>& angles);

// Helper functions to manipulate information from state vector

// Get position
inline Vector3d get_p(const VectorXd& x_t) {
  return x_t.segment(0, 3);
}

// Set position
inline void set_p(VectorXd& x_t, const Vector3d& p_t) {
  x_t.segment(0, 3) = p_t;
}

// Get orientation (wxyz)
inline Vector4d get_q(const VectorXd& x_t) {
  return x_t.segment(3, 4);
}

// Set orientation (wxyz)
inline void set_q(VectorXd& x_t, const Vector4d& q_t) {
  x_t.segment(3, 4) = q_t;
}

// Get angular velocity
inline Vector3d get_w(const VectorXd& x_t) {
  return x_t.segment(7, 3);
}

// Set angular velocity
inline void set_w(VectorXd& x_t, const Vector3d& w_t) {
  x_t.segment(7, 3) = w_t;
}

// Get acceleration
inline Vector3d get_a(const VectorXd& x_t) {
  return x_t.segment(10, 3);
}

// Set acceleration
inline void set_a(VectorXd& x_t, const Vector3d& a_t) {
  x_t.segment(10, 3) = a_t;
}

// Get ith joint velocity
inline double get_theta_dot(const VectorXd& x_t, size_t i) {
  return x_t(13 + i);
}

// Set ith joint velocity
inline void set_theta_dot(VectorXd& x_t, size_t i, double theta_dot) {
  x_t(13 + i) = theta_dot;
}

// Helper functions to manipulate information from sensor vector

// Get ith accelerometer vector
inline Vector3d get_alpha(const VectorXd& z_t, size_t i) {
  return z_t.segment(3*i, 3);
}

// Set ith accelerometer vector
inline void set_alpha(VectorXd& z_t, const VectorXd& alpha_t, size_t i) {
  z_t.segment(3*i, 3) = alpha_t;
}

// Get ith gyro vector
inline Vector3d get_gamma(const VectorXd& z_t, size_t i,
                          size_t num_modules) {
  return z_t.segment(3*num_modules + 3*i, 3);
}

// Set ith gyro vector
inline void set_gamma(VectorXd& z_t, const Vector3d& gamma_t, size_t i,
                      size_t num_modules) {
  z_t.segment(3*num_modules + 3*i, 3) = gamma_t;
}
