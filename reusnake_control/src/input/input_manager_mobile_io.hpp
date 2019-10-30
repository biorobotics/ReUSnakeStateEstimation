#pragma once

#include <Eigen/Dense>
#include <memory>
#include <atomic>
#include "input_manager.hpp"

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_feedback.hpp"

namespace hebi {
namespace input {

// If the I/O board/app is not found, command vectors return 0, and button
// um toggle/quit states are unchanged by 'update'. Class is not
// re-entrant.
class InputManagerMobileIO : public InputManager
{
public:
  InputManagerMobileIO();
  virtual ~InputManagerMobileIO() noexcept = default;

  // Connect to an I/O board/app and start getting feedback.  Return "true" if
  // found. Clears any existing connection
  bool reset();

  // A debug command that can be used to print the current state of the joystick
  // variables that are stored by the class.
  void printState() const override;

  double getRightVertRaw() const override;
  double getLeftVertRaw() const override;
  double getRightHorzRaw() const override;
  double getLeftHorzRaw() const override;
  double getSlide4() const override;
  double getModeNum() const override;

  bool shouldMove() const override;
  int getcurControlButton() const override;
  void resetCurControlButton() override;
  
  // Get the current translation velocity command
  Eigen::Vector3f getTranslationVelocityCmd() const override;

  // Get the current rotation velocity command
  Eigen::Vector3f getRotationVelocityCmd() const override;

  Eigen::VectorXd getDirection() const override;

  // Returns true if the quit button has ever been pushed
  bool getQuitButtonPushed() const override;

  // Gets the number of times the mode button has been toggled since the last
  // request. Resets this count after retrieving.
  size_t getAndResetModeToggleCount() override;

  // Is the joystick connected?
  // Return "true" if we are connected to an I/O board/app; false otherwise.
  bool isConnected() const override {
    return group_ ? true : false;
  }

private:

  float getVerticalVelocity() const;

  // The Mobile IO app that serves as a joystick
  std::shared_ptr<hebi::Group> group_;

  // Scale the joystick scale to motion of the robot in SI units (m/s, rad/s,
  // etc).
  static constexpr float xyz_scale_{0.175};
  static constexpr float rot_scale_{0.4};

  float left_horz_raw_{0}; // Rotation
  float left_vert_raw_{0}; // Chassis tilt

  float slider_1_raw_{0}; // Height
  float slider_4_raw_{0}; // Step Height

  float right_horz_raw_{0}; // Translation (l/r)
  float right_vert_raw_{0}; // Translation (f/b)

  bool prev_mode_button_state_{false};      // Mode
  std::atomic<size_t> num_mode_toggles_{0}; // 

  bool is_B1_pushed_ = false; // B1 current state
  bool prev_B1_state_{false}; // B1 prev state

  bool is_B2_pushed_ = false; // B2
  bool prev_B2_state_{false}; // B2

  bool is_B3_pushed_ = false; // B3
  bool prev_B3_state_{false}; // B3

  bool is_B4_pushed_ = false; // B4
  bool prev_B4_state_{false}; // B4

  bool is_B5_pushed_ = false; // B5
  bool prev_B5_state_{false}; // B5

  bool is_B6_pushed_ = false; // B6
  bool prev_B6_state_{false}; // B6

  int curControlButton_ = 0;

  bool has_quit_been_pushed_ = false; // Quit
};

} // namespace input
} // namespace hebi

