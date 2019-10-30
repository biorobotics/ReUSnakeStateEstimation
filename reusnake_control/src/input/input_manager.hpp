#pragma once

#include <Eigen/Dense>

namespace hebi {
namespace input {

// Input manager base class
class InputManager
{
public:
  InputManager() = default;
  virtual ~InputManager() noexcept = default;

  // A debug command that can be used to print the current state of the joystick
  // variables that are stored by the class.
  virtual void printState() const = 0;

  // Read/poll/etc; not needed for all joysticks.  Should be called periodically
  // by the application.
  virtual bool update()
  {
    // indicates joystick connected, not that there were updates
    return true;
  }

  virtual double getRightHorzRaw() const = 0;
  virtual double getLeftHorzRaw() const = 0;
  virtual double getRightVertRaw() const = 0;
  virtual double getLeftVertRaw() const = 0;
  virtual double getSlide4() const = 0;
  virtual double getModeNum() const = 0;

  virtual bool shouldMove() const = 0;
  virtual int getcurControlButton() const = 0;
  virtual void resetCurControlButton() = 0;

  // Get the current translation velocity command
  virtual Eigen::Vector3f getTranslationVelocityCmd() const = 0;

  // Get the current rotation velocity command
  virtual Eigen::Vector3f getRotationVelocityCmd() const = 0;

  virtual Eigen::VectorXd getDirection() const = 0;

  // Returns true if the quit button has ever been pushed
  virtual bool getQuitButtonPushed() const = 0;

  // Gets the number of times the mode button has been toggled since the last
  // request.
  virtual size_t getAndResetModeToggleCount() = 0;

  // Is the joystick connected?
  virtual bool isConnected() const = 0;
};

} // namespace input
} // namespace hebi

