/// \file SnakeKinematics.cpp
///
/// Basic snake kinematics functions; these create arrays of 4x4 transforms
/// that define the necesssary transform to get from a base from to any module
/// frame.
///
/// \author mtesch, xf

#include "SnakeKinematics.h"
#include <stdexcept>

/// Get a set of 4x4 transforms that define the snake in the head module
/// frame.
///
/// \param angles A vector of angles that represent the joint angles of the
///   robot.  Assumes angles are ordered from front of the robot to back.
/// \param getTransform A function pointer that takes in a double (the angle
///   to transform) and an integer (the joint number), and returns the
///   4x4 relative module-to-module transform.
///
/// \return An array of 4x4 transforms to each module (starting with the head),
///   in the head module frame.
///
/// \author xf, mtesch
transformArray makeSnake(vector<double> angles, Matrix4d (*getTransform)(double, int))
{
  // Store the transforms that will be returned:
  transformArray transforms = transformArray(angles.size() + 1);

  // The first module (head frame) will just be the identity matrix, as it is
  // our output frame:
  transforms[0] = Matrix4d::Identity();

  // Compute the transforms for subsequent modules:
  for (unsigned int joint = 0; joint < angles.size(); joint++)
  {
    // Get the transform at the current joint:
    transforms[joint+1] = transforms[joint] *
                          getTransform(angles[joint], joint);
  }

  return transforms;
}

/// Right multiply the set of 4x4 snake body frames by a given 4x4 transform.
///
/// \param moduleFrames A reference to a vector of module transforms
///   returned by makeSnake or makeSnakeInFrame; modifies this value in place.
/// \param transform A 4x4 transform to right multiply each moduleFrame by.
///
/// \author xf, mtesch
void transformSnake(transformArray &moduleFrames, Matrix4d transform)
{
  for (unsigned int i = 0; i < moduleFrames.size(); i++)
  {
    moduleFrames[i] *= transform;
  }
}

/// Left multiply the set of 4x4 snake body frames by a given 4x4 transform.
///
/// \param moduleFrames A reference to a vector of module transforms
///   returned by makeSnake or makeSnakeInFrame; modifies this value in place.
/// \param transform A 4x4 transform to left multiply each moduleFrame by.
///
/// \author xf, mtesch
void leftTransformSnake(transformArray &moduleFrames, Matrix4d transform)
{
  for (unsigned int i = 0; i < moduleFrames.size(); i++)
  {
    moduleFrames[i] = transform * moduleFrames[i];
  }
}

/// Get a set of 4x4 transforms that define the snake in the frame of a
/// given module.
///
/// \param angles A vector of angles that represent the joint angles of the
///   robot.  Assumes angles are ordered from front of the robot to back.
/// \param moduleFrame The frame of the module to transform the snake to;
///   '0' represents the head module, '1' the next module, etc.
/// \param getTransform A function pointer that takes in a double (the angle
///   to transform) and an integer (the joint number), and returns the
///   4x4 relative module-to-module transform.
///
/// \throws std::out_of_range exception if an invalid module frame is requested,
///   where invalid is out of the range 0 to angles.size().
///
/// \author xf, mtesch
transformArray makeSnakeInFrame(vector<double> angles, int moduleFrame, Matrix4d (*getTransform)(double, int))
{
  // Check for valid module frame:
  if (moduleFrame < 0 || ((unsigned int) moduleFrame) > angles.size())
    throw out_of_range("Invalid Frame Chosen");

  // Create the transforms in frame of module 0
  transformArray transforms = makeSnake(angles, getTransform);

  // Tranform the transforms by the inverse of module moduleFrame's transform
  transformSnake(transforms, transforms[moduleFrame].inverse());

  return transforms;
}

////////////////////////////////////////////////////////////////////////////////
// Unified snake kinematics functions
////////////////////////////////////////////////////////////////////////////////

/// A helper function to get the relative transform between two modules in the
/// unified snake.
///
/// \param angle The joint angle between these two modules.
/// \param joint The number of the joint (to account for spiraling).
Matrix4d getUnifiedSnakeTransform(double angle, int joint)
{
  // Define the dimensions of the unified snake.
  double moduleLength = 0.051054; //2.01 inches, in meters
  //double moduleRadius = 2.05;

  // Build translation transform that moves half of a module length back
  // along the snake.
  Matrix4d translate = Matrix4d::Identity();
  translate(2,3) = -moduleLength / 2;
  
  // Compute the rotation matrix
  Matrix4d R = Matrix4d::Identity();
  
  R.block<3,3>(0,0) = rotZ(-M_PI/2)*rotY(-angle);

  // First, translate by half of a module, rotate, then translate by half of
  // a module.
  return translate * R * translate;
}

/// Creates an array of 4x4 transforms for the unified snake in the head frame,
/// based on the angles in the 'angles' vector.
///
/// \param angles A vector of angles that represent the joint angles of the
/// robot.  Assumes angles are ordered from front of the robot to back.
///
/// \author xf, mtesch
transformArray makeUnifiedSnake(vector<double> angles)
{
  return makeSnake(angles, getUnifiedSnakeTransform);
}

/// Creates an array of 4x4 transforms for the unified snake based on the angles
/// in the 'angles' vector, in a given frame (0 is the head frame, can request
/// from 0 up to angles.size()).
///
/// \param angles A vector of angles that represent the joint angles of the
///   robot.  Assumes angles are ordered from front of the robot to back.
/// \param moduleFrame The frame of the module to transform the snake to;
///   '0' represents the head module, '1' the next module, etc.
///
/// \author xf, mtesch
transformArray makeUnifiedSnakeInFrame(vector<double> angles, int moduleFrame)
{
  return makeSnakeInFrame(angles, moduleFrame, getUnifiedSnakeTransform);
}

////////////////////////////////////////////////////////////////////////////////
// SEA snake kinematics functions
////////////////////////////////////////////////////////////////////////////////

/// A helper function to get the relative transform between two modules in the
/// SEA snake.
///
/// \param angle The joint angle between these two modules.
/// \param joint The number of the joint (to account for spiraling).
Matrix4d getSEASnakeTransform(double angle, int joint)
{
  // Snake dimensions:
  // (.0639 meters is 2.516 inches)
  // double moduleLength = 2.516;
  double inputToJoint = 1.441;
  double jointToOutput = 1.075;

  // Build translation transform to move back along the snake
  Matrix4d translateIn = Matrix4d::Identity();
  Matrix4d translateOut = Matrix4d::Identity();
  translateIn(2,3) = -inputToJoint;
  translateOut(2,3) = -jointToOutput;

  // Compute the rotation matrix, depending on rotation about the x or y
  // angle:
  Matrix4d R = Matrix4d::Identity();
  
  // Rotate by the negative angle, because we are going head to tail.
  angle = -angle;

  if (joint%2 == 0) { // Even equals X, Uneven Y
    R.block<3,3>(0,0) = rotX(angle);
  } else {
    R.block<3,3>(0,0) = rotY(angle);
  }

  // Translate, rotate, then translate to find entire transform.
  return translateOut * R * translateIn;
}

/// Creates an array of 4x4 transforms for the SEA snake in the head frame,
/// based on the angles in the 'angles' vector.
///
/// \param angles A vector of angles that represent the joint angles of the
/// robot.  Assumes angles are ordered from front of the robot to back.
///
/// \author mtesch
transformArray makeSEASnake(vector<double> angles)
{
  return makeSnake(angles, getSEASnakeTransform);
}

/// Creates an array of 4x4 transforms for the SEA snake based on the angles
/// in the 'angles' vector, in a given frame (0 is the head frame, can request
/// from 0 up to angles.size()).
///
/// \param angles A vector of angles that represent the joint angles of the
///   robot.  Assumes angles are ordered from front of the robot to back.
/// \param moduleFrame The frame of the module to transform the snake to;
///   '0' represents the head module, '1' the next module, etc.
///
/// \author mtesch
transformArray makeSEASnakeInFrame(vector<double> angles, int moduleFrame)
{
  return makeSnakeInFrame(angles, moduleFrame, getSEASnakeTransform);
}

/// Averages all of the accelerometer values from a number of frames, and return
/// a single vector in the reference frame of the transforms.
///
/// \param moduleFrames A vector of 4x4 transforms that represent the snake
///   configuration.  NOTE: These should be in the same frame as the virtual
///   chassis is represented in, e.g. everything in the head module frame!
/// \param xAccel A vector of x accelerometer readings, in the local frame of
///   each module.  Should have length of len(moduleFrames) - 1, and no NaNs.
/// \param yAccel A vector of y accelerometer readings, in the local frame of
///   each module.  Should have length of len(moduleFrames) - 1, and no NaNs.
/// \param zAccel A vector of z accelerometer readings, in the local frame of
///   each module.  Should have length of len(moduleFrames) - 1, and no NaNs.
///
/// \author mtesch
Vector3d averageAccelerometers(const transformArray &moduleFrames,
                               const std::vector<double> &xAccel,
                               const std::vector<double> &yAccel,
                               const std::vector<double> &zAccel)
{
  // Number of used accelerometers
  int accelLen = xAccel.size();

  // A vector to store the averaged accelerometer values in:
  Vector4d averageAccel = Vector4d(0, 0, 0, 0);

  // Transform the accelerometer reading from each module frame into the
  // common frame and add the together.
  for (unsigned int i = 0; i < accelLen; i++)
  {
    Vector4d accelModule = Vector4d(xAccel[i], yAccel[i], zAccel[i], 0);
    // Because the head is represented in the module frames, but doesn't
    // have an accelerometer, we need to offset the module frame transform
    // by one (hence the i+1):
    averageAccel += moduleFrames[i+1] * accelModule;
  }

  // Extract out the x, y, and z portions of this vector, and divide by the
  // number of sensor readings:
  Vector3d accelGlobal = Vector3d(averageAccel[0] / accelLen,
                                  averageAccel[1] / accelLen,
                                  averageAccel[2] / accelLen);

  return accelGlobal;
}

/// Averages all of the gyro values from a number of frames, and return
/// a single vector in the reference frame of the transforms.
///
/// \param moduleFrames A vector of 4x4 transforms that represent the snake
///   configuration.  NOTE: These should be in the same frame as the virtual
///   chassis is represented in, e.g. everything in the head module frame!
/// \param angleVelocities A vector of the angular velocities corresponding
///   to each module. Should have length of len(moduleFrames) - 1, and no NaNs.
/// \param xGyro A vector of x gyro readings, in the local frame of
///   each module.  Should have length of len(moduleFrames) - 1, and no NaNs.
/// \param yGyro A vector of y gyro readings, in the local frame of
///   each module.  Should have length of len(moduleFrames) - 1, and no NaNs.
/// \param zGyro A vector of z gyro readings, in the local frame of
///   each module.  Should have length of len(moduleFrames) - 1, and no NaNs.
///
/// \author Florian Enner < enner @ cmu.edu >
Vector3d averageGyros(const transformArray &moduleFrames,
                      const std::vector<double> &angleVelocities,
                      const std::vector<double> &xGyro,
                      const std::vector<double> &yGyro,
                      const std::vector<double> &zGyro ){

  // Extract the size of the vectors
  int gyroLen = min(angleVelocities.size(),xGyro.size());

  // Initialize vectors
  Vector4d averageGyro      (0, 0, 0, 0);
  Vector4d gyroAngVelCumComp(0, 0, 0, 0);
  Vector4d angVelVector     (0, 0, 0, 0);

  // Transform the gyro reading from each module frame into the
  // common frame and add them together. This is needed to compensate
  // for the angular velocities that get introduced by joint motion.
  // We accumulate motion from each joint in the variable gyroAngVelCumComp.
  for (unsigned int i = 0; i < gyroLen; i++)
  {
    // Check if the module rotates about the x or y direction
    // and negate the angular velocity readings induced by 
    // joint movement
    if( i%2 == 0 ){ 
      
      // even = x axis
      angVelVector = Vector4d(angleVelocities[i],0,0,0);
    
    }else{ 
      // odd = y axis
      angVelVector = Vector4d(0,angleVelocities[i],0,0);

    }
    
    // Compensate for the angular velocities of all modules up to the
    // current module. Because the head is represented in the module
    // frames, but doesn't have a gyro, we need to offset the module
    // frame transform by one (hence the i+1).
    gyroAngVelCumComp += moduleFrames[i+1] * angVelVector;

    // Construct a vector of gyro readings for the current module
    Vector4d gyroModule = Vector4d(xGyro[i], yGyro[i], zGyro[i], 0);
    
    // Rotate the gyros into the global frame, compensate angular
    // velocities and add the result to the average. The frames
    // are again offset by 1 because the head is not represented
    // in the gyro vector.
    averageGyro += moduleFrames[i+1] * gyroModule + gyroAngVelCumComp;
    
  }

  // Extract out the x, y, and z portions of this vector, and divide by the
  // number of sensor readings:
  Vector3d gyroGlobal = Vector3d( averageGyro[0] / gyroLen,
                                  averageGyro[1] / gyroLen,
                                  averageGyro[2] / gyroLen);

  return gyroGlobal;
}

/// Returns a 3x3 rotation matrix about the x-axis
///
/// \param angle The angle of rotation in radians. 
///
/// \author Florian Enner < enner @ cmu.edu > 
Matrix3d rotX(double angle){
  
  // Calculate sin/cos
  double s = sin(angle);
  double c = cos(angle);
  
  // Create matrix
  Matrix3d rot;
  rot << 1,  0,  0,
         0,  c, -s,
         0,  s,  c; 
  
  // Return
  return rot;
}

/// Returns a 3x3 rotation matrix about the y-axis
///
/// \param angle The angle of rotation in radians. 
///
/// \author Florian Enner < enner @ cmu.edu > 
Matrix3d rotY(double angle){
  
  // Calculate sin/cos
  double s = sin(angle);
  double c = cos(angle);
  
  // Create matrix
  Matrix3d rot;
  rot << c,  0,  s,
         0,  1,  0,
        -s,  0,  c;
  
  // Return
  return rot;
}

/// Returns a 3x3 rotation matrix about the z-axis
///
/// \param angle The angle of rotation in radians. 
///
/// \author Florian Enner < enner @ cmu.edu > 
Matrix3d rotZ(double angle){
  
  // Calculate sin/cos
  double s = sin(angle);
  double c = cos(angle);
  
  // Create matrix
  Matrix3d rot;
  rot << c, -s,  0,
         s,  c,  0,
         0,  0,  1;
  
  // Return
  return rot;
}

/// Returns the corresponding euler angles for a given rotation. The
/// convention used is Z,Y,Z. The returned vector elements describe
///  [0] yaw   (+/- 180 deg) in [rad]
///  [1] pitch (+/-  90 deg) in [rad]
///  [2] roll  (+/- 180 deg) in [rad]
/// rotation matrix = rotZ(yaw) * rotY(pitch) * rotX(roll)
/// 
/// TODO: clean up comments within function
///
/// \param rot A 3x3 rotation matrix
///
/// \author Florian Enner < enner @ cmu.edu > 
Vector3d getEulerAngles(Matrix3d rot){

  Vector3d alignVec = rot.transpose() * Vector3d(0,0,1);
  double s, c, roll, pitch, yaw;
  
  /// ****************************************************
  /// Calculate Roll: Rotate the positive z axis (world
  /// frame) into the local frame. In our convention roll
  /// describes the rotation about the x axis.
  /// ****************************************************
  
  // Find the angle about the z-axis in the xy-plane between the x-axis
  // of the head frame and the avg accelerometer vector.
  roll = -atan2( alignVec[1], alignVec[0] );

  // Create rotation matrix about the z axis
  Matrix3d Rz_roll = rotZ(roll); 
  
  // Align the accelerometer vector with the x-axis of the head frame
  alignVec = Rz_roll * alignVec;
  
  
  /// ****************************************************
  /// Calculate Pitch (align positive x axis (head frame)
  /// with avg accelerometer vector projected into the XZ
  /// plane by rotating about Y-axis)
  /// ****************************************************
  
  // Find rotation about y-axis (pitch) that aligns the x-axis of 
  // the rolled head-frame with the gravity vector in the head frame.
  // TODO: should it be negative alignVec[2] ?
  pitch = atan2( alignVec[2], alignVec[0] );
  
  // Create rotation matrix about the y axis
  Matrix3d Ry = rotY(pitch);
  
  
  /// ****************************************************
  /// Calculate Yaw 
  /// ****************************************************
  // NOTE: yaw * pitch * roll * roll_inverse * pitch_inverse = yaw
  // Yaw: 1,  0,  0,
  //      0,  c, -s,
  //      0,  s,  c; 
  
  // Calculate yaw rotation matrix
  Matrix3d Rz_yaw = rot * Rz_roll.transpose() * Ry.transpose();

  // Extract angle by looking at the sin and cosine values.
  yaw = -atan2( ( Rz_yaw(1,2) - Rz_yaw(2,1) ) / 2 ,  // Averaged sin part
                ( Rz_yaw(1,1) + Rz_yaw(2,2) ) / 2 ); // Averaged cos part
  
  
  /// ****************************************************
  /// Get rid of errors close to 0 
  /// ****************************************************
  double upper = 0.9999 * M_PI;
  double lower = 0.0001 * M_PI;
  
  // Above upper or below lower bound, set to 0
  yaw   = (fabs(yaw)   < lower) || (fabs(yaw)   > upper) ? 0 : yaw;
  pitch = (fabs(pitch) < lower) || (fabs(pitch) > upper) ? 0 : pitch;
  roll  = (fabs(roll)  < lower) || (fabs(roll)  > upper) ? 0 : roll;
  
  // report negative angle
  return Vector3d(yaw, pitch, roll);
}

/// Removes the yaw (rotation about z) part from a given rotation matrix.
/// The convention used is Z,Y,Z. The result describes
/// R = rotZ(yaw).inverse() * rotZ(yaw) * rotY(pitch) * rotX(roll)
///
/// \param rot A 3x3 rotation matrix
///
/// \author Florian Enner < enner @ cmu.edu > 
Matrix3d removeYaw(Matrix3d rot){
  
  // Find Euler Angles (z_yaw, y_pitch, z_roll) convention
  Vector3d ea = getEulerAngles(rot);
  double yaw = ea[0]; // first element corresponds to yaw
  
  // Rotate by the negative angle (same as inverse)
  Matrix3d rot_yaw = rotZ(-yaw);
        
  // Remove Yaw part (yaw.inverse() * yaw * pitch * roll)
  return rot_yaw * rot;
}

