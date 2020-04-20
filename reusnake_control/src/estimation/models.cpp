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
static const double lambda = 1;

// Gravitational field
static const double g = 9.8;

// Perturbation used for numerical derivatives
static const double epsilon = 0.000001;

static const double module_rad = 0.026; 
static const double zthresh = 0.0075; // for motion model

// Get state transition matrix for quaternion
void quaternion_stm(Matrix4d& stm, const Vector3d& w_t, double dt) {
  Matrix4d omega; // skew-symmetric matrix used in orientation update
  omega <<            0, -w_t(0), -w_t(1), -w_t(2),
               w_t(0),         0,  w_t(2), -w_t(1),
               w_t(1), -w_t(2),         0,  w_t(0),
               w_t(2),  w_t(1), -w_t(0),         0;
  double wmag = w_t.norm(); // magnitude of angular velocity
  double s = 0.5*dt*wmag;

  if (s == 0) {
    stm = Matrix4d::Identity();
  } else {
    stm = Matrix4d::Identity()*cos(s) + omega*sin(s)/wmag;
  }
}

void get_head_kinematics(Vector3d& accel, Vector3d& ang_vel, const VectorXd& x_t,
                         size_t num_modules, double dt,
                         short body_frame_module, const Matrix4d& prev_vc,
                         const vector<double>& angles) {
  vector<double> prev_angles(angles);
  vector<double> next_angles(angles);
  for (size_t i = 0; i < num_modules; i++) {
    // Use joint velocities to estimate angles forward and backward in time
    double dtheta = get_theta_dot(x_t, i, num_modules)*dt;
    prev_angles[i] -= dtheta;
    next_angles[i] += dtheta;
  }

  // Compute snake kinematics (module frames with respect to head frame)
  transformArray transforms = makeSEASnake(angles);

  // We need the transforms at the previous and next time step
  // for the gyro prediction
  transformArray prev_transforms = makeSEASnake(prev_angles);
  transformArray next_transforms = makeSEASnake(next_angles);

  Matrix3d body_R;
  Matrix3d prev_body_R;
  Matrix3d next_body_R;

  Vector3d body_pos = transforms[body_frame_module].block<3, 1>(0, 3);
  Vector3d prev_body_pos = prev_transforms[body_frame_module].block<3, 1>(0, 3);
  Vector3d next_body_pos = prev_transforms[body_frame_module].block<3, 1>(0, 3);

  // Negative module means virtual chassis is the body frame
  if (body_frame_module < 0) {
    // Compute virtual chassis
    Matrix4d vc = getSnakeVirtualChassis(transforms);
    Matrix4d old_vc = getSnakeVirtualChassis(transforms);
    Matrix4d next_vc = getSnakeVirtualChassis(transforms);

    makeVirtualChassisConsistent(prev_vc, vc);
    makeVirtualChassisConsistent(vc, old_vc);
    makeVirtualChassisConsistent(vc, next_vc);

    body_R = vc.block<3, 3>(0, 0);
    prev_body_R = old_vc.block<3, 3>(0, 0);
    next_body_R = next_vc.block<3, 3>(0, 0);

    body_pos = vc.block<3, 1>(0, 3);
    prev_body_pos = old_vc.block<3, 1>(0, 3);
    next_body_pos = next_vc.block<3, 1>(0, 3);  
  } else {
    body_R = transforms[body_frame_module].block<3, 3>(0, 0);
    prev_body_R= prev_transforms[body_frame_module].block<3, 3>(0, 0);
    next_body_R = next_transforms[body_frame_module].block<3, 3>(0, 0);

    body_pos = transforms[body_frame_module].block<3, 1>(0, 3);
    prev_body_pos = prev_transforms[body_frame_module].block<3, 1>(0, 3);
    next_body_pos = prev_transforms[body_frame_module].block<3, 1>(0, 3);  
  }
    
  // Invert orientation quaternion and convert it into rotation matrix
  Vector4d qvec = get_q(x_t);
  Quaterniond q_inv(qvec(0), -qvec(1), -qvec(2), -qvec(3));
 
  // Rotation matrix of world frame with respect to body frame
  Matrix3d V_t(q_inv);

  // Perceived acceleration due to gravity in world frame
  Vector3d gvec(0, 0, g);

  // Perceived acceleration due to gravity in head frame
  Vector3d a_grav = body_R*V_t*gvec;

  // Double differentiate body frame position in head frame for
  // "internal acceleration" of head frame wrt body frame
  // (represented in the head frame)
  Vector3d a_internal = -body_R*(next_body_R.transpose()*next_body_pos -
                          2*body_R.transpose()*body_pos + 
                          prev_body_R.transpose()*prev_body_pos)/(dt*dt);

  // Acceleration of body frame in world frame
  Vector3d body_accel = get_a(x_t);
  
  // Acceleration of body frame in head frame
  Vector3d a_body = body_R*V_t*body_accel;

  // Populate head acceleration
  accel = a_grav + a_internal + a_body;
  
  /* Angular velocity calculations */

  // Orientations of head wrt body frame
  Matrix3d head_R = body_R.transpose();
  Matrix3d prev_head_R = prev_body_R.transpose();

  Matrix3d velocity_matrix = prev_head_R*head_R.transpose()/dt;
  Vector3d w_internal(velocity_matrix(2, 1), velocity_matrix(0, 2), velocity_matrix(1, 0));

  // Compute angular velocity of body frame in module frame
  Vector3d w_t = get_w(x_t);

  // Populate gyro values
  ang_vel = w_internal + body_R*w_t;
} 

Vector3d get_body_displacement(const vector<double>& angles, const vector<double>& prev_angles,
                               const Matrix3d& R_vc_world, 
                               size_t num_modules, const Matrix4d& prev_vc) {
  transformArray transforms = makeSEASnake(angles);
  transformArray prev_transforms = makeSEASnake(prev_angles);

  // Compute virtual chassis
  Matrix4d body = getSnakeVirtualChassis(transforms);
  Matrix4d prev_body = getSnakeVirtualChassis(prev_transforms);

  makeVirtualChassisConsistent(prev_vc, body);
  makeVirtualChassisConsistent(body, prev_body);

  leftTransformSnake(transforms, body.inverse());
  leftTransformSnake(prev_transforms, prev_body.inverse());

  // Taken straight from "Simplified Motion Modeling" paper
  vector<Vector3d> pi(num_modules);
  vector<double> zi(num_modules);
  vector<double> wi(num_modules);

  double z_min = 0;

  // Vector that points down wrt world frame, represented in the virtual chassis frame
  Vector3d d(0, 0, -module_rad);
  d = R_vc_world.transpose()*d;

  for (size_t i = 0; i < num_modules; i++) {
    pi[i] = transforms[i + 1].block<3, 1>(0, 3) - 
                      prev_transforms[i + 1].block<3, 1>(0, 3);

    Matrix3d prev_Ri = prev_transforms[i + 1].block<3, 3>(0, 0);

    Matrix3d omega_i = prev_Ri.transpose()*transforms[i + 1].block<3, 3>(0, 0);
    
    Vector3d ri = prev_Ri.transpose()*d;

    pi[i] += prev_Ri*(omega_i - omega_i.transpose())*ri/2;

    Vector3d pos = R_vc_world*transforms[i + 1].block<3, 1>(0, 3);
    zi[i] = pos(2);
    if (zi[i] < z_min) {
      z_min = zi[i];
    }
  }

  double wsum = 0;
  for (size_t i = 0; i < num_modules; i++) {
    double diff = zi[i] - z_min;
    double gamma_i = (diff < zthresh) ? (1 - diff/zthresh) : 0;

    wi[i] = (1 - exp(-gamma_i))/(1 - exp(-1));
    wsum += wi[i];
  }

  Vector3d disp(0, 0, 0);
  for (size_t i = 0; i < num_modules; i++) {
    disp -= (wi[i]/wsum)*pi[i];
  }
  return R_vc_world*disp;
}

void f(VectorXd& x_t, const VectorXd& x_t_1, const VectorXd& u_t,
       double dt, size_t num_modules, short body_frame_module, const Matrix4d& prev_vc,
       const vector<double>& angles) {
  // Predict acceleration
  Vector3d a_t_1 = get_a(x_t_1);
  set_a(x_t, exp(-tau*dt)*a_t_1);

  Vector3d w_t_1 = get_w(x_t_1);
  
  // Predict angular velocity
  if (body_frame_module < 0) {
    // If using virtual chassis, use constant velocity model
    set_w(x_t, w_t_1);
  }
  else {
    // Otherwise, use gyro measurement
    set_w(x_t, u_t.tail(3));
  }
  
  // Predict orientation
  Matrix4d stm;
  quaternion_stm(stm, w_t_1, dt);

  Vector4d q_t_1 = get_q(x_t_1);

  Vector4d q_t;
  q_t = stm*q_t_1; //current orientation
  q_t = q_t/q_t.norm();

  set_q(x_t, q_t);

  vector<double> prev_angles(angles);
  for (size_t i = 0; i < num_modules; i++) {
    double prev_theta_dot = get_theta_dot(x_t_1, i, num_modules);
    prev_angles[i] -= prev_theta_dot*dt;
  
    set_theta_dot(x_t, i,
                  lambda*u_t(i) + (1 - lambda)*prev_theta_dot);
  }

  if (body_frame_module < 0) {
    Vector3d p_t_1 = get_p(x_t_1);
    Quaterniond q(q_t_1(0), q_t_1(1), q_t_1(2), q_t_1(3));

    Vector3d p_t = p_t_1 + get_body_displacement(angles, prev_angles, q.toRotationMatrix(),
                                                 num_modules, prev_vc);
    set_p(x_t, p_t);
  } else {
    set_p(x_t, Vector3d::Zero());
  }
}

Matrix4d h(VectorXd& z_t, const VectorXd& x_t, double dt, size_t num_modules,
           short body_frame_module, const Matrix4d& prev_vc,
           const vector<double>& angles) {
  vector<double> prev_angles(angles);
  vector<double> next_angles(angles);

  /* Predict joint angle measurements */
  for (size_t i = 0; i < num_modules; i++) {
    // Use joint velocities to estimate angles forward and backward in time
    double dtheta = get_theta_dot(x_t, i, num_modules)*dt;
    prev_angles[i] -= dtheta;
    next_angles[i] += dtheta;
  }

  // Compute snake kinematics (module frames with respect to head frame)
  transformArray transforms = makeSEASnake(angles);

  // We need the transforms at the previous and next time step
  // for the gyro prediction
  transformArray prev_transforms = makeSEASnake(prev_angles);
  transformArray next_transforms = makeSEASnake(next_angles);

  // Body frame transformations with respect to head
  Matrix4d body;
  Matrix4d prev_body;
  Matrix4d next_body;

  if (body_frame_module < 0) {
    // Compute virtual chassis
    body = getSnakeVirtualChassis(transforms);
    prev_body = getSnakeVirtualChassis(prev_transforms);
    next_body = getSnakeVirtualChassis(next_transforms);

    makeVirtualChassisConsistent(prev_vc, body);
    makeVirtualChassisConsistent(body, prev_body);
    makeVirtualChassisConsistent(body, next_body);
  } else {
    body = transforms[body_frame_module];
    prev_body = prev_transforms[body_frame_module];
    next_body = next_transforms[body_frame_module];
  }

  // Compute everything in body frame, since that's the frame
  // whose orientation we're measuring
  leftTransformSnake(transforms, body.inverse());
  leftTransformSnake(prev_transforms, prev_body.inverse());
  leftTransformSnake(next_transforms, next_body.inverse());

  /* Predict accelerometer and gyro values */

  // Invert orientation quaternion and convert it into rotation matrix
  Vector4d qvec = get_q(x_t);
  Quaterniond q_inv(qvec(0), -qvec(1), -qvec(2), -qvec(3));
 
  // Rotation matrix of world frame with respect to body frame
  Matrix3d V_t(q_inv);

  // Perceived acceleration due to gravity in world frame
  Vector3d gvec(0, 0, g);

  Vector3d w_t = get_w(x_t);

  for (size_t i = 0; i < num_modules; i++) {
    /* Accelerometer calculations */

    // Rotation matrix of body frame with respect to current module frame
    Matrix3d R_inv = transforms[i + 1].block<3, 3>(0, 0).transpose();

    // Perceived acceleration due to gravity in module frame
    Vector3d a_grav = R_inv*V_t*gvec;

    // Double differentiate module position in body frame for
    // "internal acceleration" in module frame
    Vector3d position = transforms[i + 1].block<3, 1>(0, 3);
    Vector3d prev_position = prev_transforms[i + 1].block<3, 1>(0, 3);
    Vector3d next_position = next_transforms[i + 1].block<3, 1>(0, 3);

    Vector3d a_internal = R_inv*(next_position - 2*position + prev_position)/(dt*dt);

    // Acceleration of body frame in world frame
    Vector3d body_accel = get_a(x_t);
    
    // Acceleration of body frame in module frame
    Vector3d a_body = R_inv*V_t*body_accel;
    
    // Populate accelerometer measurement
    Vector3d alpha_t = a_grav + a_body + a_internal;
    set_alpha(z_t, alpha_t, i);
    
    /* Gyro calculations */

    // Current and previous rotation matrix of module frame with respect to body frame
    Matrix3d R = transforms[i + 1].block<3, 3>(0, 0);
    Matrix3d prev_R = prev_transforms[i + 1].block<3, 3>(0, 0);

    Matrix3d velocity_matrix = prev_R.transpose()*R/dt;
    Vector3d w_internal(velocity_matrix(2, 1), velocity_matrix(0, 2), velocity_matrix(1, 0));

    // Compute angular velocity of body frame in module frame
    Vector3d w_t_module = R_inv*w_t;

    // Populate gyro values
    Vector3d gamma_t = w_internal + w_t_module;

    set_gamma(z_t, gamma_t, i, num_modules);
  }

  if (body_frame_module < 0) {
    return body;
  }
}

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
        const vector<double>& angles) {
  size_t statelen = state_length(num_modules);

  F_t.setZero();

  // Jacobian of position wrt previous position
  F_t.block<3, 3>(0, 0).setIdentity();

  if (body_frame_module < 0) {
    vector<double> prev_angles(angles);
    for (size_t i = 0; i < num_modules; i++) {
      prev_angles[i] -= get_theta_dot(x_t_1, i, num_modules)*dt;
    }

    // Jacobian of position wrt error angle via chain rule
    
    // Compute Jacobian of quaternion wrt error angle
    Vector4d q_t_1 = get_q(x_t_1);
    MatrixXd dq_ddtheta(4, 3);
    dq_ddtheta << -q_t_1(1), -q_t_1(2), -q_t_1(3),
                  q_t_1(0), -q_t_1(3), q_t_1(2),
                  q_t_1(3), q_t_1(0), -q_t_1(1),
                  -q_t_1(2), q_t_1(1), q_t_1(0);

    dq_ddtheta *= 0.5;

    MatrixXd dp_dq(3, 4);
    for (size_t i = 0; i < 4; i++) {
      Vector4d q_t_1_plus(q_t_1); 
      Vector4d q_t_1_minus(q_t_1); 
      q_t_1_plus(i) += epsilon;
      q_t_1_minus(i) -= epsilon;
      Quaterniond q_plus(q_t_1_plus(0), q_t_1_plus(1), q_t_1_plus(2), q_t_1_plus(3));
      Quaterniond q_minus(q_t_1_minus(0), q_t_1_minus(1), q_t_1_minus(2), q_t_1_minus(3));
      
      Vector3d disp_plus = get_body_displacement(angles, prev_angles, q_plus.toRotationMatrix(),
                                                 num_modules, prev_vc);
      Vector3d disp_minus = get_body_displacement(angles, prev_angles, q_minus.toRotationMatrix(),
                                                 num_modules, prev_vc);
      
      dp_dq.col(i) = (disp_plus - disp_minus)/(2*epsilon);
    }
    F_t.block<3, 3>(0, 3) = dp_dq*dq_ddtheta;

    Quaterniond q(q_t_1(0), q_t_1(1), q_t_1(2), q_t_1(3));
    Matrix3d R = q.toRotationMatrix();
    // Numerical Jacobian wrt angular velocities
    for (size_t i = 0; i < num_modules; i++) {
      vector<double> prev_angles_plus(prev_angles);
      vector<double> prev_angles_minus(prev_angles);

      prev_angles_plus[i] -= epsilon*dt;
      prev_angles_minus[i] += epsilon*dt;

      Vector3d disp_plus = get_body_displacement(angles, prev_angles_plus, R,
                                                 num_modules, prev_vc);
      Vector3d disp_minus = get_body_displacement(angles, prev_angles_minus, R,
                                                 num_modules, prev_vc);

      F_t.block<3, 1>(0, 12 + i) = (disp_plus - disp_minus)/(2*epsilon);
    }
  } else {
    F_t.block(0, 3, 3, statelen - 4).setZero();
  }

  // Acceleration is only dependent on previous acceleration
  F_t.block<3, 3>(9, 9) = exp(-tau*dt)*Matrix3d::Identity();

  Vector3d w_t_1 = get_w(x_t_1);

  // Jacobian of error angle with respect to previous
  Vector3d delta_theta = w_t_1*dt;
  double theta = delta_theta.norm();

  if (theta == 0) {
    F_t.block<3, 3>(3, 3).setIdentity();
  }
  else {
    AngleAxisd ang_ax(theta, delta_theta/theta);
    F_t.block<3, 3>(3, 3) = ang_ax.toRotationMatrix().transpose();
  }

  // Account for virtual chassis constant velocity model
  if (body_frame_module < 0) {
    F_t.block<3, 3>(6, 6) = Matrix3d::Identity();
  }

  MatrixXd I = MatrixXd::Identity(num_modules, num_modules);

  // Joint velocity Jacobian
  F_t.block(12, 12, num_modules, num_modules) = (1 - lambda)*I;
}

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
        const vector<double>& angles) {
  size_t statelen = state_length(num_modules);
  size_t sensorlen = sensor_length(num_modules);

  // Use chain rule to evaluate Jacobian wrt error quaternion
  MatrixXd dz_dq(sensorlen, 4);

  H_t.block(0, 0, sensorlen, 3).setZero();

  for (size_t col = 3; col < statelen; col++) {
    // Perturb state variable corresponding to this column of J
    VectorXd x_t_plus(x_t);
    VectorXd x_t_minus(x_t);
    x_t_plus(col) += epsilon;
    x_t_minus(col) -= epsilon;

    VectorXd z_t_plus(sensorlen);
    h(z_t_plus, x_t_plus, dt, num_modules, body_frame_module, prev_vc, angles);
    VectorXd z_t_minus(sensorlen);
    h(z_t_minus, x_t_minus, dt, num_modules, body_frame_module, prev_vc, angles);

    // Compute derivative
    VectorXd dz_t = (z_t_plus - z_t_minus)/(2*epsilon);

    if (col < 7) {
      dz_dq.col(col - 3) = dz_t;
    } else {
      H_t.col(col - 1) = dz_t;
    }
  }

  // Compute Jacobian of quaternion wrt error angle
  Vector4d q_t = get_q(x_t);
  MatrixXd dq_ddtheta(4, 3);
  dq_ddtheta << -q_t(1), -q_t(2), -q_t(3),
                q_t(0), -q_t(3), q_t(2),
                q_t(3), q_t(0), -q_t(1),
                -q_t(2), q_t(1), q_t(0);

  dq_ddtheta *= 0.5;

  // Apply chain rule
  H_t.block(0, 3, sensorlen, 3) = dz_dq*dq_ddtheta;
}

Matrix4d init_state(VectorXd& x_t, const VectorXd& z_t, size_t num_modules,
                    short body_frame_module, const vector<double>& angles) {
  // We start with joint velocities of zero
  x_t = VectorXd::Zero(state_length(num_modules));

  // Use the body frame module's initial orientation using its accelerometer
  // (or the module 1 accelerometer if we're using the virtual chassis)
  transformArray transforms = makeSEASnake(angles);

  Vector3d a_grav(0, 0, g); // gravitational acceleration in world frame

  Vector3d a_grav_body; // gravitational acceleration in body frame

  Matrix4d body;
  if (body_frame_module < 0) {
    body = getSnakeVirtualChassis(transforms);
    a_grav_body = get_alpha(z_t, 0);
  } else {
    body = transforms[body_frame_module];
    a_grav_body = get_alpha(z_t, body_frame_module - 1);
  }
  cout << a_grav_body << endl;

  // Rotation matrix representing body frame in world frame
  Matrix3d R = Quaterniond::FromTwoVectors(a_grav_body, a_grav).toRotationMatrix(); 

  if (body_frame_module < 0) {
    R = R*transforms[1].block<3, 3>(0, 0).transpose()*body.block<3, 3>(0, 0);
  }

  // Set head yaw to 0
  Matrix3d head_R = R*body.block<3, 3>(0, 0).transpose();
  Vector3d head_ypr = head_R.eulerAngles(2, 1, 0);
  Matrix3d new_head_R = rotY(head_ypr(1))*rotX(head_ypr(2));
  Matrix3d new_R = new_head_R*body.block<3, 3>(0, 0);

  Quaterniond new_q(new_R);

  Vector4d qvec(new_q.w(), new_q.x(), new_q.y(), new_q.z());
  
  set_q(x_t, qvec);

  if (body_frame_module < 0) {
    return body;
  }
}
