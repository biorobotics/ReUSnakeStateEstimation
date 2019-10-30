/**
  ******************************************************************************
  * @file    SRUKF.cpp
  * @author  YANG Shuo
  * @version V1.1.0
  * @date    20-Jan-2019
  * @version V1.0.0
  * @date    02-April-2014
  * @brief   This file provides SRUKF functions initialization, predict and correct 
  *           
  ******************************************************************************  
  */ 


#include "ukf.hpp"

SRUKF::SRUKF(int _n, int _m, 
             double _q, double  _r,
             void (*_f_func)(VectorXd& x_t, const VectorXd& x_t_1, 
                const Vector3d a, const Vector3d w, const std::vector<Eigen::VectorXd> ee_vel_c_list, const double dt), 
             void (*_h_func)(VectorXd&, const VectorXd&), 
             double _w_m0, double _w_c0)
{
    n = _n;
    m = _m;
    f_func = _f_func;
    h_func = _h_func;

    R = _r * MatrixXd::Identity(n, n);
    Q = _q * MatrixXd::Identity(m, m);

    w_m0 = _w_m0;
    w_c0 = _w_c0;

    w_mi = 0.5 * (1-w_m0)/(double)n;
    w_ci = 0.5 * (1-w_c0)/(double)n;

    gamma = sqrt((double)n/(1-w_m0));

    state_pre.resize(n);
    state_post.resize(n);

    S_pre = MatrixXd::Identity(n,n);
    S_post = MatrixXd::Identity(n,n);

    //cout << w_mi << " - " << w_ci << " - " << gamma << endl << endl;
}

SRUKF::SRUKF(int _n, int _m, 
			 double posStd_R, double ortStd_R, double imuStd_R,
			 double posStd_Q, double ortStd_Q, double imuStd_Q,
             void (*_f_func)(VectorXd& x_t, const VectorXd& x_t_1, 
                const Vector3d a, const Vector3d w, const std::vector<Eigen::VectorXd> ee_vel_c_list, const double dt), 
             void (*_h_func)(VectorXd&, const VectorXd&), 
             double _w_m0, double _w_c0)
{
    n = _n;
    m = _m;
    f_func = _f_func;
    h_func = _h_func;

    w_m0 = _w_m0;
    w_c0 = _w_c0;

    w_mi = 0.5 * (1-w_m0)/(double)n;
    w_ci = 0.5 * (1-w_c0)/(double)n;

    gamma = sqrt((double)n/(1-w_m0));

    R = MatrixXd::Identity(n, n);
    Q = MatrixXd::Identity(m, m);
	for (int i = 0; i < 3; i++)
    	R(i,i) = posStd_R;
	for (int i = 3; i < 7; i++)
    	R(i,i) = ortStd_R;
	for (int i = 7; i < 13; i++)
    	R(i,i) = imuStd_R;
	for (int i = 0; i < 3; i++)
    	Q(i,i) = posStd_Q;
	for (int i = 3; i < 7; i++)
    	Q(i,i) = ortStd_Q;
	for (int i = 7; i < 13; i++)
    	Q(i,i) = imuStd_Q;

    state_pre.resize(n);
    state_post.resize(n);

    S_pre = MatrixXd::Identity(n,n);
    S_post = MatrixXd::Identity(n,n);
}
void SRUKF::setR(double _posStd_R, double _ortStd_R, double _imuStd_R)
{
	for (int i = 0; i < 3; i++)
    	R(i,i) = _posStd_R;
	for (int i = 3; i < 7; i++)
    	R(i,i) = _ortStd_R;
	for (int i = 7; i < 13; i++)
    	R(i,i) = _imuStd_R;
}

void SRUKF::setR(MatrixXd _R)
{
	R=_R;
}

void SRUKF::setQ(double _posStd_Q, double _ortStd_Q, double _imuStd_Q)
{
	for (int i = 0; i < 3; i++)
    	Q(i,i) = _posStd_Q;
	for (int i = 3; i < 7; i++)
    	Q(i,i) = _ortStd_Q;
	for (int i = 7; i < 13; i++)
    	Q(i,i) = _imuStd_Q;
}

void SRUKF::setQ(MatrixXd _Q)
{
	Q=_Q;
}

void SRUKF::setMeasurementFunc(void (*_h_func)(VectorXd&, const VectorXd&), int measure_size)
{
	h_func=_h_func;
    m = measure_size;
}

void SRUKF::predict(const Vector3d a, const Vector3d w, const std::vector<Eigen::VectorXd> ee_vel_c_list, const double dt, const float phase)
{
    //TODO: check if state_post is empty or not, 
    //      currently just assume state_post is ready to use
    // cout << "predict start ===========================" << endl;
    sigmaPoints.resize(0,0);
    sigmaPoints = state_post.replicate(1,2*n+1);
    // cout << "sOSigmaPoints:\n" << sigmaPoints << endl << endl;
    sigmaPoints.block(0,1,n,n) += gamma * S_post;
    sigmaPoints.block(0,n+1,n,n) -= gamma * S_post;

    // cout << "sigmaPoints:\n" << sigmaPoints << endl << endl;
    x_tmp.resize(n);
    for (int i = 0; i < 2*n+1; i++)
    {
        f_func(x_tmp, sigmaPoints.col(i), a, w, ee_vel_c_list, dt);
        sigmaPoints.col(i) = x_tmp;
    }
    // cout << "sigmaPoints:\n" << sigmaPoints << endl << endl;

    state_pre = w_m0* sigmaPoints.col(0);
    for (int i = 1; i < 2*n+1; i++)
        state_pre += w_mi * sigmaPoints.col(i);

    OS = MatrixXd(n, 3*n);

    OS.block(0,0,n,2*n) = sqrt(w_ci)*(sigmaPoints.block(0,1,n,2*n)
                          - state_pre.replicate(1,2*n));


    OS.block(0,2*n,n,n) = Q; 

    // cout << "OS:\n" << OS << endl << endl;
    // cout <<"reach " << __FILE__ << __LINE__ << endl;
    qrSolver.compute(OS.transpose());
    m_q = qrSolver.householderQ();
    m_r = qrSolver.matrixQR().triangularView<Upper>();

    // cout <<"reach " << __FILE__ << __LINE__ << endl;
    //cout << "QR error: " << (m_q*m_r - OS.transpose()).norm() << endl << endl;

    S_pre = m_r.block(0,0,n,n).transpose();
    // cout <<"reach " << __FILE__ << __LINE__ << endl;
    //cout << "S_pre before update:\n" << S_pre << endl << endl;
    //cout << "sigma point 0" << sigmaPoints.col(0).transpose() << endl << endl; 
    //cout << "state_pre:\n" << state_pre.transpose() << endl << endl; 

    if (w_c0 < 0)
        internal::llt_inplace<double,Upper>::rankUpdate(S_pre, 
            sqrt(-w_c0)*(sigmaPoints.col(0) - state_pre), 
            -1);
    else
        internal::llt_inplace<double,Upper>::rankUpdate(S_pre, 
            sqrt(w_c0)*(sigmaPoints.col(0) - state_pre), 
            1);
    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    for (int i = 1; i < n; i++)
        for (int j = 0; j < i; j++)
            S_pre(i,j) = 0;
    // cout <<"reach " << __FILE__ << __LINE__ << endl;
    /*
    if (w_c0 < 0)
        internal::ldlt_inplace<Lower>::updateInPlace(S_pre, sqrt(-w_c0)*(sigmaPoints.col(0) - state_pre), -1);
    else
        internal::ldlt_inplace<Lower>::updateInPlace(S_pre, sqrt(w_c0)*(sigmaPoints.col(0) - state_pre), 1);
    */
    //cout << "S_pre after update: \n" << S_pre << endl << endl; 
    // cout << "predict end ===========================" << endl;
}

void SRUKF::correct(const VectorXd& z_t, const float phase)
{
    // normalize quaternion
    Quaterniond q(state_post(0),state_post(1),state_post(2),state_post(3));
    q.normalize();
    state_post.segment<4>(0) = q.coeffs();
    // cout << "correct ===========================" << endl;
    sigmaPoints.resize(0,0);
    sigmaPoints = state_pre.replicate(1,2*n+1);

    sigmaPoints.block(0,1,n,n) += gamma * S_pre;
    sigmaPoints.block(0,n+1,n,n) -= gamma * S_pre;

    state_pre = w_m0 * sigmaPoints.col(0);
    for (int i = 1; i < 2*n+1; i++)
        state_pre += w_mi * sigmaPoints.col(i);

    z_tmp.resize(m);
    sigmaZ.resize(m, 2*n+1);
    for (int i = 0; i < 2*n+1; i++)
    {
        h_func(z_tmp, sigmaPoints.col(i)); 
        sigmaZ.col(i) = z_tmp;
    }

    // cout << "sigmaZ\n" << sigmaZ << endl << endl;

    z_t_bar = w_m0*sigmaZ.col(0); 
    for (int i = 1; i < 2*n+1; i++)
        z_t_bar += w_mi * sigmaZ.col(i);


    // cout << "z_t_bar\n" << z_t_bar << endl << endl;

    OS = MatrixXd(m, 2*n+m);
    OS.block(0,0,m,2*n) = sqrt(w_ci)*(sigmaZ.block(0,1,m,2*n)
                          - z_t_bar.replicate(1,2*n));

    
    OS.block(0,2*n,m,m) = R; 

    // cout << "OS:\n" << OS << endl << endl;
    qrSolver.compute(OS.transpose());
    m_r = qrSolver.matrixQR().triangularView<Upper>();

    S_y_bar = m_r.block(0,0,m,m).transpose();

    // cout << "S_y_bar\n" << S_y_bar << endl << endl;
    if (w_c0 < 0)
        internal::llt_inplace<double,Upper>::rankUpdate(S_y_bar, 
            sqrt(-w_c0)*(sigmaZ.col(0) - z_t_bar), 
            -1);
    else
        internal::llt_inplace<double,Upper>::rankUpdate(S_y_bar, 
            sqrt(w_c0)*(sigmaZ.col(0) - z_t_bar), 
            1);
    for (int i = 1; i < m; i++)
        for (int j = 0; j < i; j++)
            S_y_bar(i,j) = 0;

    P_t_xz_bar = w_c0 * (sigmaPoints.col(0) - state_pre) 
                      * (sigmaZ.col(0) - z_t_bar).transpose(); 
    for (int i = 1; i< 2*n+1; i++)
    {
        P_t_xz_bar += w_ci * (sigmaPoints.col(i) - state_pre) 
                          * (sigmaZ.col(i) - z_t_bar).transpose(); 
    }

    // cout << "P_t_xz_bar\n" << P_t_xz_bar << endl << endl;
    qrSolver.compute(S_y_bar);
    K_tmp = qrSolver.solve(P_t_xz_bar.transpose());
    
    qrSolver.compute(S_y_bar.transpose());
    K_t = qrSolver.solve(K_tmp).transpose();

    // cout << "K_t\n" << K_t << endl << endl;

    state_post = state_pre + K_t*(z_t - z_t_bar);

    U = K_t * S_y_bar;
    S_post = S_pre;

    for (int i = 0; i<m;i++)
        internal::llt_inplace<double,Lower>::rankUpdate(S_post, 
            U.col(i), 
            -1);
    for (int i = 1; i < n; i++)
        for (int j = 0; j < i; j++)
            S_post(i,j) = 0;

    // cout << "state_post: \n" << state_post << endl << endl; 
    // cout << "S_post: \n" << S_post << endl << endl; 
}