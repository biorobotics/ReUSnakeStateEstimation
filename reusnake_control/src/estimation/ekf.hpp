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
         * modules: number of modules in snake
         * _body_frame_module: module number of body frame, or -1 for virtual chassis
         * z_t: current measurement
         */
        void initialize(size_t modules, short _body_frame_module, const VectorXd& z_t);

        MatrixXd Q; // processs covariance
		    MatrixXd R; // measurement covariance
        
        VectorXd x_t; // current state estimate
		    MatrixXd S_t; // prediction covariance

        VectorXd h_t; // predicted measurement, for printing mainly

        Matrix4d vc; // current virtual chassis

    private:
        size_t num_modules; //number of modules in snake
        short body_frame_module; // module number of body frame, or -1 for virtual chassis
      
        size_t statelen; //length of state vector
        size_t sensorlen; //length of sensor vector

        double dt; // time increment used for last estimate
};
