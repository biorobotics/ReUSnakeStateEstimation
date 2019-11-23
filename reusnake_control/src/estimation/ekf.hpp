/**
  ***************************************************************************************
  * @file    ekf.hpp
  * @author  Anoop Bhat
  * @version V1.0.0
  * @date    04-Nov-2019
  * @brief   This file is an implementation of an EKF for a snake robot
  ***************************************************************************************
  */
#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class EKF
{
    public:
        /* EKF: constructor for extended Kalman filter with diagonal covariance
                matrices
         * ARGUMENTS
         * q: sensor noise/covariance
         * r: process noise/covariance
         * _f_func: process model function
         * _h_func: sensor model function
         * _state_length: function to compute length of state vector
         * _sensor_length: function to compute length of sensor vector
         * modules: number of modules in snake
         */
        EKF(double q, double r,
            void (*_f_func)(VectorXd& x_t, const VectorXd& x_t_1,
                            const VectorXd& u_t, double dt,
                            size_t num_modules),
            void (*_h_func)(VectorXd& z_t, const VectorXd& x_t, double dt,
                            size_t num_modules),
            size_t (*_state_length)(size_t num_modules),
            size_t (*_sensor_length)(size_t num_modules),
            void (*init_state)(VectorXd& x_t, const VectorXd& z_t,
                               size_t num_modules),
            size_t modules);
        
        /* predict: runs predict step
         * ARGUMENTS
         * u_t: control signal
         * dt: time between x_t_1 and x_t
         */
        void predict(const VectorXd& u_t, double _dt);
      
        /* correct: runs correction step with the same dt as the prediction
         * step run immediately beforehand
         * ARGUMENTS
         * z_t: current measurement
         */
        void correct(const VectorXd& z_t);
        
        /* initialize: sets initial value for state using sensor values
         * ARGUMENTS
         * z_t: current measurement
         */
        void initialize(const VectorXd& z_t);

        MatrixXd R; // measurement covariance
		    MatrixXd Q; // processs covariance
        
        VectorXd x_t; // current state estimate
		    MatrixXd S_t; // prediction covariance

    private:
        size_t num_modules; //number of modules in snake
      
        size_t state_length; //length of state vector
        size_t sensor_length; //length of sensor vector

        double dt; // time increment used for last estimate

        // function pointer to process function
        void (*f_func)(VectorXd& x_t, const VectorXd& x_t_1, 
                       const VectorXd& u_t,
                       double dt, size_t num_modules);
        // function pointer to measurement function
        void (*h_func)(VectorXd& z_t, const VectorXd& x_t, double dt,
                       size_t num_modules);               
        // function pointer to state initialization function
        void (*init_state)(VectorXd& x_t, const VectorXd& z_t, 
                           size_t num_modules);               
        /*
         * df: computes numerical Jacobian of f by perturbing each state variable
         * by epsilon
         * F_t: Jacobian of f
         * x_t_1: state to evaluate Jacobian
         * u_t: control signal
         */
        void df(MatrixXd& F_t, VectorXd x_t_1, const VectorXd& u_t);

        /*
         * dh: computes numerical Jacobian of h by perturbing each state variable
         * by epsilon
         * H_t: Jacobian of h
         * x_t: state to evaluate Jacobian
         */
        void dh(MatrixXd& H_t, VectorXd x_t);
};
