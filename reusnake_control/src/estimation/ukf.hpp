/**
  ***************************************************************************************
  * @file    SRUKF.hpp
  * @author  YANG Shuo
  * @version V1.1.0
  * @date    20-Jan-2019
  * @version V1.0.0
  * @date    02-April-2014
  * @brief   This file is a implementation of square-root UKF, depends on linear algebra
  *          C++ library Eigen3  
  ***************************************************************************************
  */
#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class SRUKF
{
    public:

        SRUKF(int _n, int _m, 
              double _q, double  _r,
              void (*_f_func)(VectorXd& x_t, const VectorXd& x_t_1, 
                const Vector3d a, const Vector3d w, const std::vector<Eigen::VectorXd> ee_vel_c_list, const double dt), 
              void (*_h_func)(VectorXd&, const VectorXd&), 
              double _w_m0 = 0.5, double _w_c0 = 0.5);

        // another constructor, must use along with setR and setQ
        SRUKF(int _n, int _m, 
			  double posStd_R, double ortStd_R, double imuStd_R,
			  double posStd_Q, double ortStd_Q, double imuStd_Q,
              void (*_f_func)(VectorXd& x_t, const VectorXd& x_t_1, 
                const Vector3d a, const Vector3d w, const std::vector<Eigen::VectorXd> ee_vel_c_list, const double dt), 
              void (*_h_func)(VectorXd&, const VectorXd&), 
              double _w_m0 = 0.5, double _w_c0 = 0.5);

        void setR(double _posStd_R, double _ortStd_R, double _imuStd_R);
        void setQ(double _posStd_Q, double _ortStd_Q, double _imuStd_Q);

        void setR(MatrixXd _R);
        void setQ(MatrixXd _Q);

        void predict(const Vector3d a, const Vector3d w, const std::vector<Eigen::VectorXd> ee_vel_c_list,  const double dt, const float phase);
        void correct(const VectorXd& z_t, const float phase);
		    void setMeasurementFunc(void (*_h_func)(VectorXd&, const VectorXd&), int measure_size);

        VectorXd state_pre, state_post;
        MatrixXd S_pre, S_post;
        
        MatrixXd R; // measurement errorCov
		    MatrixXd Q; // processs errorCov

    private:
        int n;              //state dimension
        int m;              //measurement dimension

        double w_m0, w_mi;
        double w_c0, w_ci;   //ukf weights
        double gamma;        // ukf parameter

        MatrixXd K_t;

        // function pointer to process function
        void (*f_func)(VectorXd& x_t, const VectorXd& x_t_1, 
            const Vector3d a, const Vector3d w, const std::vector<Eigen::VectorXd> ee_vel_c_list, const double dt);   
        // function pointer to measurement function
        void (*h_func)(VectorXd&, const VectorXd&);               

        MatrixXd sigmaPoints;
        MatrixXd sigmaZ;

        // Eigen function for QR decomposition
        HouseholderQR<MatrixXd> qrSolver;
        FullPivHouseholderQR<MatrixXd> colPiv_qrSolver;
        MatrixXd m_q, m_r;
        // tmp variables for computation
        VectorXd x_tmp, z_tmp, z_t_bar;
        MatrixXd OS;
        MatrixXd S_y_bar, P_t_xz_bar;
        MatrixXd K_tmp;
        MatrixXd U;

};