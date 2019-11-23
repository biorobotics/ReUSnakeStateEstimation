/// \file SnakeKinematics.h
///
/// Basic snake kinematics functions; these create arrays of 4x4 transforms
/// that define the necesssary transform to get from a base from to any module
/// frame.
///
/// \author mtesch, xf

#ifndef SNAKEKINEMATICS_H
#define SNAKEKINEMATICS_H

#include "Eigen/Dense"
#include "Eigen/StdVector"
#include <vector>

using namespace Eigen;
using namespace std;

/// NOTE: This is a typedef to simplify messy notation when using fixed-size
/// Eigen objects in std::vector classes.
///
/// See http://eigen.tuxfamily.org/dox/TopicStlContainers.html for more
/// explanation on why this is necessary.
typedef vector< Matrix4d, aligned_allocator<Matrix4d> > transformArray;


/// Creates an array of 4x4 transforms for the unified snake in the head frame,
/// based on the angles in the 'angles' vector.
///
/// \param angles A vector of angles that represent the joint angles of the
/// robot.  Assumes angles are ordered from front of the robot to back.
///
/// \author xf, mtesch
transformArray makeUnifiedSnake(vector<double> angles);

/// Creates an array of 4x4 transforms for the SEA snake in the head frame,
/// based on the angles in the 'angles' vector.
///
/// \param angles A vector of angles that represent the joint angles of the
/// robot.  Assumes angles are ordered from front of the robot to back.
///
/// \author mtesch
transformArray makeSEASnake(vector<double> angles);

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
transformArray makeUnifiedSnakeInFrame(vector<double> angles, int moduleFrame);

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
transformArray makeSEASnakeInFrame(vector<double> angles, int moduleFrame);

/// Right multiply the set of 4x4 snake body frames by a given 4x4 transform.
///
/// \param moduleFrames A reference to a vector of module transforms
///   returned by makeSnake or makeSnakeInFrame; modifies this value in place.
/// \param transform A 4x4 transform to right multiply each moduleFrame by.
///
/// \author xf, mtesch
void transformSnake(transformArray &moduleFrames, Matrix4d transform);

/// Left multiply the set of 4x4 snake body frames by a given 4x4 transform.
///
/// \param moduleFrames A reference to a vector of module transforms
///   returned by makeSnake or makeSnakeInFrame; modifies this value in place.
/// \param transform A 4x4 transform to left multiply each moduleFrame by.
///
/// \author xf, mtesch
void leftTransformSnake(transformArray &moduleFrames, Matrix4d transform);

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
                               const std::vector<double> &zAccel);
       
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
                      const std::vector<double> &zGyro );

/// Returns a 3x3 rotation matrix about the x-axis
///
/// \param angle The angle of rotation in radians. 
///
/// \author Florian Enner < enner @ cmu.edu > 
Matrix3d rotX(double angle);

/// Returns a 3x3 rotation matrix about the y-axis
///
/// \param angle The angle of rotation in radians. 
///
/// \author Florian Enner < enner @ cmu.edu > 
Matrix3d rotY(double angle);

/// Returns a 3x3 rotation matrix about the z-axis
///
/// \param angle The angle of rotation in radians. 
///
/// \author Florian Enner < enner @ cmu.edu > 
Matrix3d rotZ(double angle);

/// Returns the corresponding euler angles for a given rotation. The
/// convention used is Z,Y,Z. The returned vector elements describe
///  [0] yaw   (+/- 180 deg) in [rad]
///  [1] pitch (+/-  90 deg) in [rad]
///  [2] roll  (+/- 180 deg) in [rad]
/// rotation matrix = rotZ(yaw) * rotY(pitch) * rotX(roll)
///
/// \param rot A 3x3 rotation matrix
///
/// \author Florian Enner < enner @ cmu.edu > 
Vector3d getEulerAngles(Matrix3d rot);

/// Removes the yaw (rotation about z) part from a given rotation matrix.
/// The convention used is Z,Y,Z. The result describes
/// R = rotZ(yaw).inverse() * rotZ(yaw) * rotY(pitch) * rotX(roll)
///
/// \param rot A 3x3 rotation matrix
///
/// \author Florian Enner < enner @ cmu.edu > 
Matrix3d removeYaw(Matrix3d);
  
  
#endif 
