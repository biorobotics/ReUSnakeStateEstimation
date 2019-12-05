#include <Eigen/Dense>
#include <cmath>
#include "SnakeKinematics.h"
#include "VirtualChassisKinematics.h"
#include "models.hpp"
#include <iostream>

/*
 * Some notes on the frame transformations used here:
 * 1. It appears that the z-axis for each joint points along
 * the snake's backbone in the SnakeKinematics frames
 * 2. SnakeKinematics gives us num_modules + 1 frames (to include
 * the head frame)
 */

// Damping term for acceleration
static const double tau = 21;

// How much weight we give the commanded joint velocities over the previous 
// joint velocities in the update step
static const double lambda = 0.25;

// Gravitational field
static const double g = 9.8;

// Perturbation used for numerical derivatives
static const double epsilon = 0.001;

// Get state transition matrix for quaternion
void quaternion_stm(Matrix4d& stm, Vector3d& w_t_1, double dt) {
  Matrix4d omega; // skew-symmetric matrix used in orientation update
  omega << 0, -w_t_1(0), -w_t_1(1), -w_t_1(2),
         w_t_1(0), 0, w_t_1(2), -w_t_1(1),
         w_t_1(1), -w_t_1(2), 0, w_t_1(0),
         w_t_1(2), w_t_1(1), -w_t_1(0), 0;
  Matrix4d I = Matrix<double, 4, 4>::Identity();
  
  double wmag = sqrt(w_t_1.squaredNorm()); // magnitude of angular velocity
  double s = 0.5*dt*wmag;

  if (s == 0) {
    stm = I;
  } else {
    stm = I*cos(s) + omega*sin(s)/wmag;
  }
}

void f(VectorXd& x_t, const VectorXd& x_t_1, const VectorXd& u_t,
       double dt, size_t num_modules) {
  // Predict acceleration
  Vector3d a_t_1;
  get_a(a_t_1, x_t_1);
  set_a(x_t, exp(-tau*dt)*a_t_1);
  
  // Predict orientation
  Vector3d w_t_1; //angular velocity
  get_w(w_t_1, x_t_1);

  Matrix4d stm;
  quaternion_stm(stm, w_t_1, dt);

  Vector4d q_t_1;
  get_q(q_t_1, x_t_1);

  Vector4d q_t;
  q_t = stm*q_t_1; //current orientation
  q_t = q_t/sqrt(q_t.squaredNorm());

  set_q(x_t, q_t);

  // Predict angular velocity (constant velocity model)
  Vector3d w_t;
  get_w(w_t, x_t_1);
  set_w(x_t, w_t);

  for (size_t i = 0; i < num_modules; i++) {
    // Predict joint angles
    double prev_angle = get_theta(x_t_1, i);
    double prev_vel = get_theta_dot(x_t_1, i, num_modules);
    set_theta(x_t, i, fmod(prev_angle + prev_vel*dt, 2*M_PI));
  
    // Predict joint velocities as weighted sum of previous and commanded
    // velocities
    double v_t = (u_t(i) - get_theta(x_t_1, i))/dt;
    set_theta_dot(x_t, i, (1 - lambda)*get_theta_dot(x_t_1, i, num_modules) + lambda*v_t, num_modules);
  }
}

void h(VectorXd& z_t, const VectorXd& x_t, double dt, size_t num_modules) {
  //we'll use these for virtual chassis
  vector<double> angles(num_modules);
  vector<double> prev_angles(num_modules);
  vector<double> next_angles(num_modules);

  /* Predict joint angle measurements */
  for (size_t i = 0; i < num_modules; i++) {
    angles[i] = get_theta(x_t, i);
    set_phi(z_t, i, angles[i]);

    // Use joint velocities to estimate angles forward and backward in time
    double dtheta = get_theta_dot(x_t, i, num_modules)*dt;
    prev_angles[i] = angles[i] - dtheta;
    next_angles[i] = angles[i] + dtheta;
  }

  // Compute snake kinematics (module frames with respect to head frame)
  transformArray transforms = makeUnifiedSnake(angles);

  // We need the transforms at the previous and next time step
  // for the gyro prediction
  transformArray prev_transforms = makeUnifiedSnake(prev_angles);
  transformArray next_transforms = makeUnifiedSnake(next_angles);

  // Compute virtual chassis
  Matrix4d vc = getSnakeVirtualChassis(transforms);
  Matrix4d prev_vc = getSnakeVirtualChassis(prev_transforms);
  Matrix4d next_vc = getSnakeVirtualChassis(next_transforms);
  
  makeVirtualChassisConsistent(prev_vc, vc);
  makeVirtualChassisConsistent(vc, next_vc);
  
  // Compute snake kinematics in VC
  leftTransformSnake(transforms, vc.inverse()); 
  //leftTransformSnake(prev_transforms, prev_vc.inverse()); 
  //leftTransformSnake(next_transforms, next_vc.inverse()); 
  leftTransformSnake(prev_transforms, vc.inverse()); 
  leftTransformSnake(next_transforms, vc.inverse()); 
  
  /* Predict accelerometer and gyro values */

  // Invert orientation quaternion and convert it into rotation matrix
  Vector4d qvec;
  get_q(qvec, x_t);
  Quaterniond q_inv(qvec(0), -qvec(1), -qvec(2), -qvec(3));
 
  // Rotation matrix of world frame with respect to virtual chassis frame
  Matrix3d V_t(q_inv);

  // Perceived acceleration due to gravity in world frame
  Vector3d gvec(0, g, 0);

  for (size_t i = 0; i < num_modules; i++) {
    /* Accelerometer calculations */

    // Rotation matrix of VC with respect to current module frame
    Matrix3d R_inv;
    R_inv = transforms[i + 1].block(0, 0, 3, 3).transpose();

    // Perceived acceleration due to gravity in module frame
    Vector3d a_grav = R_inv*V_t*gvec;

    // Double differentiate module position in VC frame for
    // "internal acceleration" in module frame
    Vector3d position = transforms[i + 1].block(0, 3, 3, 1);
    Vector3d prev_position = prev_transforms[i + 1].block(0, 3, 3, 1);
    Vector3d next_position = next_transforms[i + 1].block(0, 3, 3, 1);

    Vector3d prev_velocity = (position - prev_position)/dt;
    Vector3d next_velocity = (next_position - position)/dt;

    Vector3d a_internal = R_inv*(next_velocity - prev_velocity)/dt;

    // Acceleration of VC in world frame
    Vector3d vc_accel;
    get_a(vc_accel, x_t);
    
    // Acceleration of VC in module frame
    Vector3d a_vc = R_inv*V_t*vc_accel;
    
    // Populate accelerometer measurement
    Vector3d alpha_t = a_grav + a_internal + a_vc;
    set_alpha(z_t, alpha_t, i, num_modules);
    
    /* Gyro calculations */

    // Current and previous rotation matrix of module frame with respect to VC frame
    Matrix3d R;
    Matrix3d prev_R;
    prev_R = prev_transforms[i + 1].block(0, 0, 3, 3);
    R = transforms[i + 1].block(0, 0, 3, 3);

    Matrix3d R_dot = (R - prev_R)/dt;
    Matrix3d velocity_matrix = -R*(R_dot.transpose());
    
    // Compute angular velocity of VC in module frame
    Vector3d w_t;
    get_w(w_t, x_t);
    Vector3d w_t_module = R_inv*w_t;

    // Populate gyro values
    Vector3d gamma_t;
    gamma_t(0) = velocity_matrix(2, 1) + w_t_module(0);
    gamma_t(1) = velocity_matrix(0, 2) + w_t_module(1);
    gamma_t(2) = velocity_matrix(1, 0) + w_t_module(2);
    set_gamma(z_t, gamma_t, i, num_modules);
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
  size_t statelen = state_length(num_modules);

  F_t.setZero();

  // Acceleration is only dependent on previous acceleration
  F_t.block(0, 0, 3, 3) = exp(-tau*dt)*Matrix3d::Identity();

  // Quaternion Jacobian with respect to previous quaternion
  // is simply the state transition matrix
  Vector3d w_t_1;
  get_w(w_t_1, x_t_1);
  Matrix4d stm;
  quaternion_stm(stm, w_t_1, dt);
  F_t.block(3, 3, 4, 4) = stm;

  // Compute Jacobian with respect to angular velocities numerically
  for (size_t col = 7; col < 10; col++) {
    VectorXd x_t_1_p(x_t_1); // Make a copy to perturb
    // Perturb state variable corresponding to this column of J
    x_t_1_p(col) += epsilon;
    VectorXd x_t_plus(statelen);
    f(x_t_plus, x_t_1_p, u_t, dt, num_modules);
    x_t_1_p(col) -= 2*epsilon;
    VectorXd x_t_minus(statelen);
    f(x_t_minus, x_t_1_p, u_t, dt, num_modules);
    x_t_1_p(col) += epsilon;

    // Compute derivative
    VectorXd dx_t = (x_t_plus - x_t_minus)/(2*epsilon);

    // Populate J
    for (size_t row = 3; row < 7; row++) {
      F_t(row, col) = dx_t(row);
    }
  }

  // Since we're using a constant velocity model, the derivatives
  // of the angular velocities are all 0 except for the derivatives
  // with respect to previous angular velocities
  F_t.block(7, 7, 3, 3).setIdentity();

  // Joint angles and joint velocities only depend on
  // previous joint angles and joint velocities
  
  // Joint angle Jacobian
  F_t.block(10, 10, num_modules, num_modules).setIdentity();
  F_t.block(10, 10 + num_modules, num_modules, num_modules) = 
                               dt*MatrixXd::Identity(num_modules, num_modules);

  // Joint velocity Jacobian
  F_t.block(10 + num_modules, 10, num_modules, num_modules) =
                      (-lambda/dt)*MatrixXd::Identity(num_modules, num_modules);
  F_t.block(10 + num_modules, 10 + num_modules, num_modules, num_modules) = 
                      (1 - lambda)*MatrixXd::Identity(num_modules, num_modules);
  /*
  for (size_t col = 0; col < statelen; col++) {
    VectorXd x_t_1_p(x_t_1); // Make a copy to perturb
    // Perturb state variable corresponding to this column of J
    x_t_1_p(col) += epsilon;
    VectorXd x_t_plus(statelen);
    f(x_t_plus, x_t_1_p, u_t, dt, num_modules);
    x_t_1_p(col) -= 2*epsilon;
    VectorXd x_t_minus(statelen);
    f(x_t_minus, x_t_1_p, u_t, dt, num_modules);
    x_t_1_p(col) += epsilon;

    // Compute derivative
    VectorXd dx_t = (x_t_plus - x_t_minus)/(2*epsilon);

    // Populate J
    for (size_t row = 0; row < statelen; row++) {
      F_t(row, col) = dx_t(row);
    }
  }
  */
}

/*
* dh: computes Jacobian of h
* H_t: Jacobian of h
* x_t: state to evaluate Jacobian
* dt: time step
* num_modules: number of modules in the snake
*/
void dh(MatrixXd& H_t, const VectorXd& x_t, double dt, size_t num_modules) {
  size_t statelen = state_length(num_modules);
  size_t sensorlen = sensor_length(num_modules);
  for (size_t col = 0; col < statelen; col++) {
    // Perturb state variable corresponding to this column of J
    VectorXd x_t_plus(x_t);
    VectorXd x_t_minus(x_t);
    x_t_plus(col) += epsilon;
    x_t_minus(col) -= epsilon;

    if (col == 13) {
      cout << "HERE\n";
    }
    
    VectorXd z_t_plus(sensorlen);
    h(z_t_plus, x_t_plus, dt, num_modules);
    VectorXd z_t_minus(sensorlen);
    h(z_t_minus, x_t_minus, dt, num_modules);

    // Compute derivative
    VectorXd dz_t = (z_t_plus - z_t_minus)/(2*epsilon);

    H_t.block(0, col, sensorlen, 1) = dz_t;
  }
  // Derivative of angle measurements with respect to joint angles
  // is simply 1
  H_t.block(0, 10, num_modules, num_modules).setIdentity();
}

size_t state_length(size_t num_modules) {
  return 10 + 2*num_modules;
}

size_t sensor_length(size_t num_modules) {
  return 7*num_modules;
}

void init_state(VectorXd& x_t, const VectorXd& z_t, size_t num_modules) {
  // We want to start with zero acceleration,
  // zero angular velocity, and joint velocities of zero
  x_t = VectorXd::Zero(state_length(num_modules));

  // Use measured angles to initialize angle state
  vector<double> angles(num_modules);
  for (size_t i = 0; i < num_modules; i++) {
    angles[i] = get_phi(z_t, i);
    set_theta(x_t, i, angles[i]); 
  }
  
  // We don't initialize with identity orientation. Rather,
  // use the first module's initial orientation (measured using its accelerometer)
  // and measure the initial vc orientation from that
  transformArray transforms = makeUnifiedSnake(angles);

  Matrix4d vc = getSnakeVirtualChassis(transforms);

  Vector3d a_grav_module; // module 1 accelerometer value
  get_alpha(a_grav_module, z_t, 0, num_modules);
  Vector3d a_grav(0, g, 0); // gravitational acceleration in world frame
  Quaterniond module_q; // quaternion representing module 1 in world frame
  module_q.setFromTwoVectors(a_grav_module, a_grav);
  Matrix3d module_R(module_q); // rotation matrix for module 1 in world frame
  Matrix4d module_T = Matrix4d::Identity(); // associated homogeneous transformation
  module_T.block(0, 0, 3, 3) = module_R;

  // compute vc in world frame (from left to right, we have
  // transforms from world to module 1, module 1 to head, head to vc)
  vc = module_T*transforms[1].inverse()*vc;

  Matrix3d vc_R; // rotation matrix of vc
  vc_R = vc.block(0, 0, 3, 3);
  
  Quaterniond q_t(vc_R);
  
  Vector4d qvec;
  qvec(0) = q_t.w();
  qvec(1) = q_t.x();
  qvec(2) = q_t.y();
  qvec(3) = q_t.z();
  
  set_q(x_t, qvec);
}

// Helper functions to manipulate information from state vector

// Get acceleration
void get_a(Vector3d& a_t, const VectorXd& x_t) {
  for (size_t i = 0; i < 3; i++) {
    a_t(i) = x_t(i);
  }
}

// Set acceleration
void set_a(VectorXd& x_t, const Vector3d& a_t) {
  for (size_t i = 0; i < 3; i++) {
    x_t(i) = a_t(i);
  }
}

// Get orientation (wxyz)
void get_q(Vector4d& q_t, const VectorXd& x_t) {
  for (size_t i = 0; i < 4; i++) {
    q_t(i) = x_t(3 + i);
  }
}

// Set orientation (wxyz)
void set_q(VectorXd& x_t, const Vector4d& q_t) {
  for (size_t i = 0; i < 4; i++) {
    x_t(3 + i) = q_t(i);
  }
}

// Get angular velocity
void get_w(Vector3d& w_t, const VectorXd& x_t) {
  for (size_t i = 0; i < 3; i++) {
    w_t(i) = x_t(7 + i);
  }
}

// Set angular velocity
void set_w(VectorXd& x_t, const Vector3d& w_t) {
  for (size_t i = 0; i < 3; i++) {
    x_t(7 + i) = w_t(i);
  }
}

// Get ith joint angle
double get_theta(const VectorXd& x_t, size_t i) {
  return x_t(10 + i);
}

// Set ith joint angle
void set_theta(VectorXd& x_t, size_t i, double theta) {
  x_t(10 + i) = theta;
}

// Get ith joint velocity
double get_theta_dot(const VectorXd& x_t, size_t i, size_t num_modules) {
  return x_t(10 + num_modules + i);
}

// Set ith joint velocity
void set_theta_dot(VectorXd& x_t, size_t i, double theta_dot,
                   size_t num_modules) {
  x_t(10 + num_modules + i) = theta_dot;
}

// Helper functions to manipulate information from sensor vector

// Get ith joint angle
double get_phi(const VectorXd& z_t, size_t i) {
  return z_t(i);
}

// Set ith joint angle
void set_phi(VectorXd& z_t, size_t i, double phi) {
  z_t(i) = phi;
}

// Get ith accelerometer vector
void get_alpha(Vector3d& alpha_t, const VectorXd& z_t, size_t i,
               size_t num_modules) {
  for (size_t j = 0; j < 3; j++) {
    alpha_t(j) = z_t(num_modules + 3*i + j);
  }
}

// Set ith accelerometer vector
void set_alpha(VectorXd& z_t, const VectorXd& alpha_t, size_t i,
               size_t num_modules) {
  for (size_t j = 0; j < 3; j++) {
    z_t(num_modules + 3*i + j) = alpha_t(j);
  }
}

// Get ith gyro vector
double get_gamma(Vector3d& gamma_t, const VectorXd& z_t, size_t i,
                 size_t num_modules) {
  for (size_t j = 0; j < 3; j++) {
    gamma_t(j) = z_t(4*num_modules + 3*i + j);
  }
}

// Set ith gyro vector
void set_gamma(VectorXd& z_t, const Vector3d& gamma_t, size_t i,
               size_t num_modules) {
  for (size_t j = 0; j < 3; j++) {
    z_t(4*num_modules + 3*i + j) = gamma_t(j);
  }
}

// Get head orientation (wxyz)
void get_head(Vector4d& q_head_vec, const VectorXd& x_t, size_t num_modules) {
  Vector4d qvec;
  get_q(qvec, x_t);
  Quaterniond q_t(qvec(0), qvec(1), qvec(2), qvec(3));
  Matrix3d vc_R(q_t); // rotation of VC with respect to world

  vector<double> angles(num_modules);
  for (size_t i = 0; i < num_modules; i++) {
    angles[i] = get_theta(x_t, i);
  }
  transformArray transforms = makeUnifiedSnake(angles);
  Matrix4d vc = getSnakeVirtualChassis(transforms);

  Matrix4d head_T = vc.inverse(); // transform of head with respect to VC
  Matrix3d head_R = head_T.block(0, 0, 3, 3); // rotation of head with respect to VC
  
  Matrix3d head_RW = vc_R*head_R; // rotation of head with respect to world
  Quaterniond q_head(head_RW);
  q_head_vec(0) = q_head.w();
  q_head_vec(1) = q_head.x();
  q_head_vec(2) = q_head.y();
  q_head_vec(3) = q_head.z();
}
