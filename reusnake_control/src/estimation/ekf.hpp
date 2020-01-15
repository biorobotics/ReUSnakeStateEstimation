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
         * q: process noise/covariance
         * r: sensor noise/covariance
         * modules: number of modules in snake
         */
        EKF(double q, double r, size_t modules);
  
        /* EKF: constructor for extended Kalman filter
         * ARGUMENTS
         * _Q: process noise/covariance
         * _R: sensor noise/covariance
         * _S: initial prediction covariance
         * modules: number of modules in snake
         */
        EKF(MatrixXd& _Q, MatrixXd& _R, MatrixXd& _S, size_t modules);
        
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

        MatrixXd Q; // processs covariance
		    MatrixXd R; // measurement covariance
        
        VectorXd x_t; // current state estimate
		    MatrixXd S_t; // prediction covariance

        VectorXd h_t; // predicted measurement, for printing mainly

    private:
        size_t num_modules; //number of modules in snake
      
        size_t statelen; //length of state vector
        size_t sensorlen; //length of sensor vector

        double dt; // time increment used for last estimate
};
