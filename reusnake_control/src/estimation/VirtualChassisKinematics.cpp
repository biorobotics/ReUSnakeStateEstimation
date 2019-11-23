/// \file VirtualChassisKinematics.h
///
/// Basic kinematics functions to retrieve the virtual chassis frame of the
/// robot.
///
/// \author mtesch, xf

#include "VirtualChassisKinematics.h"

/// Returns the transform representing the virtual chassis in a world reference
/// frame, based on the snake configuration represented by the transformArray.
///
/// \param moduleFrames A vector of 4x4 transforms that represent each module
///   of the snake, given in the same world reference frame.
///
/// \author xf, mtesch
Matrix4d getSnakeVirtualChassis(const transformArray &moduleFrames)
{
  // Put the translation portions of the module frame homogeneous transforms
  // into a single large matrix
  MatrixXd xyzPoints(moduleFrames.size(), 3);
  for (unsigned int i = 0; i < moduleFrames.size(); i++)
  {
    xyzPoints.row(i) = moduleFrames[i].block(0,3,3,1).transpose();
  }

  // Get the center of mass, and then zero mean the data
  Vector3d centerOfMass = xyzPoints.colwise().mean();
  for (int i = 0; i < 3; i++)
  {
    xyzPoints.col(i).array() -= centerOfMass(i);
  }

  // Take the SVD of the zero-mean positions.
  // Use thinSVD since we only need V.
  JacobiSVD<MatrixXd> svd(xyzPoints, ComputeThinU | ComputeThinV);

  // Get the singular values and V matrix
  Matrix3d V = svd.matrixV();

  // Make sure the 1st principal moment points towards the head module:
  if (V.col(0).dot( xyzPoints.row(0) ) < 0)
  {
    V.col(0) = -V.col(0);
  }

  // Ensure a right-handed frame by making the 3rd axis the cross product of
  // the first two:
  V.col(2) = V.col(0).cross(V.col(1));

  // 4x4 transformation matrix for the virtual chassis:
  Matrix4d VC = Matrix4d::Identity();
  VC.block<3,3>(0,0) = V;
  VC.block<3,1>(0,3) = centerOfMass;

  // Return the VC:
  return VC;
}

/// Alters the directions of the new virtual chassis axes to ensure that they
/// point in the same directions as the previous ones.
///
/// \param oldVC The previous virtual chassis
/// \param newVC The new virtual chassis (will be modified)
///
/// \author xf, mtesch
void makeVirtualChassisConsistent(const Matrix4d &oldVC, Matrix4d &newVC)
{
  // Make sure x-axis isn't flipped from the last frame:
  if (newVC.col(0).dot( oldVC.col(0) ) < 0)
  {
    // Rotate the VC 180 degrees about the y axis:
    Matrix4d rotateY = Matrix4d::Identity();
    rotateY.block<3,3>(0,0) = rotY(M_PI);
    newVC = newVC * rotateY;
  }

  // Make sure y-axis isn't flipped from the last frame:
  if (newVC.col(1).dot( oldVC.col(1) ) < 0)
  {
    // Rotate the VC 180 degrees about the x axis:
    Matrix4d rotateX = Matrix4d::Identity();
    rotateX.block<3,3>(0,0) = rotX(M_PI);
    newVC = newVC * rotateX;
  }
}

/// Transform the virtual chassis so the z axis lines up with the direction of
/// gravity (according to the accelerometer values passed in).
///
/// \param VC The virtual chassis coordinate system.
/// \param avgAccel The average accelerometer readings from the snake, such as
///   value returned from SnakeKinematics' averagedAccelerometer function.
///
/// \author mtesch
void setVirtualChassisAxesFromAccelerometers(Matrix4d &VC,
                                             const Vector3d &avgAccel)
{
  // Flip the direction of the accelerometers to represent gravity instead of
  // the accelerometer
  Vector3d gravityGlobal = avgAccel;

  // Align the z axis of the virtual chassis with gravity:
  // If the z-axis of the virtual chassis is not pointed in the same direction
  // as gravity, flip around the x-axis by 180 degrees:
  if (gravityGlobal.dot(Vector3d(VC(0,2), VC(1,2), VC(2,2))) < 0)
  {
    // Rotate the VC 180 degrees about the primary (x) axis:
    Matrix4d rotateX = Matrix4d::Identity();
    rotateX.block<3,3>(0,0) = rotX(M_PI);
    VC = VC * rotateX;
  }
}
