/// \file VirtualChassisKinematics.h
///
/// Basic kinematics functions to retrieve the virtual chassis frame of the
/// robot.
///
/// \author mtesch, xf

#ifndef VIRTUALCHASSISKINEMATICS_H
#define VIRTUALCHASSISKINEMATICS_H

#include "SnakeKinematics.h"

/// Returns the transform representing the virtual chassis in a world reference
/// frame, based on the snake configuration represented by the transformArray.
///
/// \param moduleFrames A vector of 4x4 transforms that represent each module
///   of the snake, given in the same world reference frame.
///
/// \author xf, mtesch
Matrix4d getSnakeVirtualChassis(const transformArray &moduleFrames);

/// Alters the directions of the new virtual chassis axes to ensure that they
/// point in the same directions as the previous ones.
///
/// \param oldVC The previous virtual chassis
/// \param newVC The new virtual chassis (will be modified)
///
/// \author xf, mtesch
void makeVirtualChassisConsistent(const Matrix4d &oldVC, Matrix4d &newVC);

/// Transform the virtual chassis so the z axis lines up with the direction of
/// gravity (according to the accelerometer values passed in).
///
/// \param VC The virtual chassis coordinate system.
/// \param avgAccel The average accelerometer readings from the snake, such as
///   value returned from SnakeKinematics' averagedAccelerometer function.
///
/// \author mtesch
void setVirtualChassisAxesFromAccelerometers(Matrix4d &VC,
                                             const Vector3d &avgAccel);

#endif
