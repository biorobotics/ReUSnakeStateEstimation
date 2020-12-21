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
 * f: predicts current state given previous state and time interval
 * (process model)
 * ARGUMENTS
 * x_t: current state, returned with the following convention
 *      - q_t: body frame orientation (quaternion [qw, qx, qy, qz])
 * x_t_1: previous state, passed with above convention
 * u_t: control signal (gyro measurement of module 1)
 * dt: time interval between x_t and x_t_1
 * num_modules: number of modules in snake
 */
void f(VectorXd& x_t, const VectorXd& x_t_1, const VectorXd& u_t,
       double dt, size_t num_modules);

/*
 * h: predicts sensor measurements given current state (measurement model)
 * ARGUMENTS
 * z_t: predicted sensor measurements, returned with the following convention
        - alpha_t: accelerometer values (vector with length 3*num_modules)
 * x_t: current state, passed with same conventions as f
 * dt: time interval between x_t and x_t_1
 * num_modules: number of modules in snake 
 */
void h(VectorXd& z_t, const VectorXd& x_t, double dt, size_t num_modules, vector<double> angles);

/*
* df: computes Jacobian of f
* F_t: Jacobian of f
* x_t_1: state to evaluate Jacobian
* u_t: control signal
* dt: time step
* num_modules: number of modules in the snake
*/
void df(MatrixXd& F_t, const VectorXd& x_t_1, const VectorXd& u_t, double dt,
        size_t num_modules);

/*
* dh: computes Jacobian of h
* H_t: Jacobian of h
* x_t: state to evaluate Jacobian
* dt: time step
* num_modules: number of modules in the snake
*/
void dh(MatrixXd& H_t, const VectorXd& x_t, double dt, size_t num_modules,
        vector<double> angles);

/*
 * state_length: computes the length of the state vector assumed by this
 *               model
 * ARGUMENTS
 * num_modules: number of modules in snake
 * RETURN
 * length of state vector that this model uses
 */
size_t state_length(size_t num_modules);

/*
 * sensor_size: computes the length of the sensor vector assumed by this
 *              model
 * ARGUMENTS
 * num_modules: number of modules in snake
 * RETURN
 * length of sensor vector that this model uses
 */
size_t sensor_length(size_t num_modules);

/*
 * init_state: initializes the state vector using sensor readings
 */
void init_state(VectorXd& x_t, const VectorXd& z_t, size_t num_modules);

// Helper functions to manipulate information from state vector

// Get orientation (wxyz)
void get_q(Vector4d& q_t, const VectorXd& x_t);

// Set orientation (wxyz)
void set_q(VectorXd& x_t, const Vector4d& q_t);

// Helper functions to manipulate information from sensor vector

// Get ith accelerometer vector
void get_alpha(Vector3d& alpha_t, const VectorXd& z_t, size_t i,
               size_t num_modules);

// Set ith accelerometer vector
void set_alpha(VectorXd& z_t, const VectorXd& alpha_t, size_t i,
               size_t num_modules);
