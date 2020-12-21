#include <Eigen/Dense>
#include <cmath>
#include "SnakeKinematics.h"
#include "models_acc.hpp"
#include <iostream>

// Damping term for acceleration
static const double tau = 21;

// Gravitational field
static const double g = 9.8;

// Perturbation used for numerical derivatives
static const double epsilon = 0.000001;

// Get state transition matrix for quaternion
void quaternion_stm(Matrix4d& stm, const Vector3d& w_t_1, double dt) {
  Matrix4d omega; // skew-symmetric matrix used in orientation update
  omega <<            0, -w_t_1(0), -w_t_1(1), -w_t_1(2),
               w_t_1(0),         0,  w_t_1(2), -w_t_1(1),
               w_t_1(1), -w_t_1(2),         0,  w_t_1(0),
               w_t_1(2),  w_t_1(1), -w_t_1(0),         0;
  Matrix4d I = Matrix<double, 4, 4>::Identity();
  
  double wmag = w_t_1.norm(); // magnitude of angular velocity
  double s = 0.5*dt*wmag;

  if (s == 0) {
    stm = I;
  } else {
    stm = I*cos(s) + omega*sin(s)/wmag;
  }
}

void f(VectorXd& x_t, const VectorXd& x_t_1, const VectorXd& u_t,
       double dt, size_t num_modules) {
  Matrix4d stm;
  quaternion_stm(stm, u_t, dt);

  Vector4d q_t_1;
  get_q(q_t_1, x_t_1);

  Vector4d q_t;
  q_t = stm*q_t_1; //current orientation
  q_t = q_t/q_t.norm();

  set_q(x_t, q_t);
}

void h(VectorXd& z_t, const VectorXd& x_t, double dt, size_t num_modules,
       vector<double> angles) {
  // Compute snake kinematics (module frames with respect to head frame)
  transformArray transforms = makeUnifiedSnake(angles);

  // Compute everything in module 1 frame, since that's the frame
  // whose orientation we're measuring
  leftTransformSnake(transforms, transforms[1].inverse());

  /* Predict accelerometer */

  // Invert orientation quaternion and convert it into rotation matrix
  Vector4d qvec;
  get_q(qvec, x_t);
  Quaterniond q_inv(qvec(0), -qvec(1), -qvec(2), -qvec(3));
 
  // Rotation matrix of world frame with respect to module 1 frame
  Matrix3d V_t = q_inv.toRotationMatrix();

  // Perceived acceleration due to gravity in world frame
  Vector3d gvec(0, 0, g);

  for (size_t i = 0; i < num_modules; i++) {
    /* Accelerometer calculations */

    // Rotation matrix of module 1 with respect to current module frame
    Matrix3d R_inv = transforms[i + 1].block(0, 0, 3, 3).transpose();

    Vector3d a_grav = R_inv*V_t*gvec;

    /*
    // Acceleration of module 1 in world frame
    Vector3d m1_accel;
    get_a(m1_accel, x_t);
    
    // Acceleration of module 1 in module frame
    Vector3d a_m1 = R_inv*V_t*m1_accel;

    Vector3d alpha_t = a_m1 + a_grav;
    
    // Populate accelerometer measurement
    set_alpha(z_t, alpha_t, i, num_modules);
    */
    set_alpha(z_t, a_grav, i, num_modules);
  }
}

/*
* df: computes Jacobian of f
* F_t: Jacobian of f
* x_t_1: state to evaluate Jacobian
* u_t: control signal
* dt: time step
* num_modules: number of modules in the snake
*/
void df(MatrixXd& F_t, const VectorXd& x_t_1, const VectorXd& u_t, double dt,
        size_t num_modules) {
  F_t.setZero();

  Vector3d delta_theta = u_t*dt;
  double theta = delta_theta.norm();
  AngleAxisd ang_ax(theta, delta_theta/theta);
  F_t = ang_ax.toRotationMatrix().transpose();
  /*
  F_t <<          1, delta_theta(2), -delta_theta(1),
                 -delta_theta(2),         1,  delta_theta(0),
                 delta_theta(1), -delta_theta(0),         1;
                 */
}

/*
* dh: computes Jacobian of h
* H_t: Jacobian of h
* x_t: state to evaluate Jacobian
* dt: time step
* num_modules: number of modules in the snake
*/
void dh(MatrixXd& H_t, const VectorXd& x_t, double dt, size_t num_modules, vector<double> angles) {
  size_t sensorlen = sensor_length(num_modules);
  for (size_t col = 0; col < 3; col++) {
    // Perturb component or error state corresponding to this column of J
    Vector3d error_state_plus = Vector3d::Zero();
    error_state_plus(col) += epsilon;

    Quaterniond q_t(x_t(0), x_t(1), x_t(2), x_t(3)); 
    double real_part =  1 - epsilon*epsilon/4;
    Quaterniond delta_q_plus;
    delta_q_plus.w() = real_part;
    delta_q_plus.vec() = error_state_plus/2;

    Quaterniond q_t_plus = q_t*delta_q_plus;
    Quaterniond q_t_minus = q_t*delta_q_plus.conjugate();

    Vector4d x_t_plus(q_t_plus.w(), 0, 0, 0);
    x_t_plus.tail(3) = q_t_plus.vec();
    Vector4d x_t_minus(q_t_minus.w(), 0, 0, 0);
    x_t_minus.tail(3) = q_t_minus.vec();

    VectorXd z_t_plus(sensorlen);
    h(z_t_plus, x_t_plus, dt, num_modules, angles);
    VectorXd z_t_minus(sensorlen);
    h(z_t_minus, x_t_minus, dt, num_modules, angles);

    // Compute derivative
    VectorXd dz_t = (z_t_plus - z_t_minus)/(2*epsilon);

    H_t.block(0, col, sensorlen, 1) = dz_t;
  }
}

size_t state_length(size_t num_modules) {
  return 4;
}

size_t sensor_length(size_t num_modules) {
  return 3*num_modules;
}

void init_state(VectorXd& x_t, const VectorXd& z_t, size_t num_modules) {
  x_t = VectorXd::Zero(state_length(num_modules));

  Vector3d a_grav_module; // module 1 accelerometer value
  get_alpha(a_grav_module, z_t, 0, num_modules);
  Vector3d a_grav(0, 0, g); // gravitational acceleration in world frame
  Quaterniond q_t; // quaternion representing module 1 in world frame
  q_t.setFromTwoVectors(a_grav_module, a_grav);

  // Set yaw to 0
  Vector3d ypr = q_t.toRotationMatrix().eulerAngles(2, 1, 0);
  Matrix3d new_rot = rotY(ypr(1))*rotX(ypr(2));

  Quaterniond new_q(new_rot);

  Vector4d qvec;
  qvec(0) = new_q.w();
  qvec.tail(3) = new_q.vec();
  
  set_q(x_t, qvec);
}

// Helper functions to manipulate information from state vector

// Get orientation (wxyz)
void get_q(Vector4d& q_t, const VectorXd& x_t) {
  for (size_t i = 0; i < 4; i++) {
    //q_t(i) = x_t(3 + i);
    q_t(i) = x_t(i);
  }
}

// Set orientation (wxyz)
void set_q(VectorXd& x_t, const Vector4d& q_t) {
  for (size_t i = 0; i < 4; i++) {
    //x_t(3 + i) = q_t(i);
    x_t(i) = q_t(i);
  }
}

// Helper functions to manipulate information from sensor vector

// Get ith accelerometer vector
void get_alpha(Vector3d& alpha_t, const VectorXd& z_t, size_t i,
               size_t num_modules) {
  for (size_t j = 0; j < 3; j++) {
    alpha_t(j) = z_t(3*i + j);
  }
}

// Set ith accelerometer vector
void set_alpha(VectorXd& z_t, const VectorXd& alpha_t, size_t i,
               size_t num_modules) {
  for (size_t j = 0; j < 3; j++) {
    z_t(3*i + j) = alpha_t(j);
  }
}
