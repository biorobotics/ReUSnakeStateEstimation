#include "input_manager_mobile_io.hpp"
#include "xml_util/pugixml.hpp"


#include <iostream> // Note: for debugging output.

namespace hebi {
namespace input {

InputManagerMobileIO::InputManagerMobileIO()
{
  if (!reset())
    std::cout << "Could not find joystick input board!" << std::endl;
}

bool InputManagerMobileIO::reset()
{
  // Cleanup if we already have one!
  if (group_)
    group_.reset();

  // Look for joystick (IO board)
  hebi::Lookup lookup;
  long timeout_ms = 2000; // use a 2 second timeout
  group_ = lookup.getGroupFromNames({"HEBI"}, {"Mobile IO"}, timeout_ms);
  if (!group_)
    return false;

  group_->addFeedbackHandler([this] (const GroupFeedback& fbk)
  {
    // TODO: use 'std::atomic' variables for state here? Add mutex?  Does this
    // slow things down due to lock contention?
    const auto& analog = fbk[0].io().a();
    const auto& digital = fbk[0].io().b();
    if (analog.hasFloat(1) && analog.hasFloat(2) && analog.hasFloat(3) &&
        digital.hasInt(1))
    {
      left_horz_raw_ = analog.getFloat(1);
      left_vert_raw_ = analog.getFloat(2);

      slider_1_raw_ = analog.getFloat(3);
      slider_4_raw_ = analog.getFloat(6);
      
      right_horz_raw_ = analog.getFloat(7);
      right_vert_raw_ = analog.getFloat(8);

      // get button 1-6 state and switch state
      bool new_B1_state = (digital.getInt(1) == 1);
      if (new_B1_state) 
        is_B1_pushed_ = true;
      else
        is_B1_pushed_ = false;
      if (new_B1_state && !prev_B1_state_) // edge trigger down
        curControlButton_ = 0;
      prev_B1_state_ = new_B1_state;

      bool new_B2_state = (digital.getInt(2) == 1);
      if (new_B2_state) 
        is_B2_pushed_ = true;
      else
        is_B2_pushed_ = false;
      if (new_B2_state && !prev_B2_state_) // edge trigger down
        curControlButton_ = 1;
      prev_B2_state_ = new_B2_state;

      bool new_B3_state = (digital.getInt(3) == 1);
      if (new_B3_state) 
        is_B3_pushed_ = true;
      else
        is_B3_pushed_ = false;
      if (new_B3_state && !prev_B3_state_) // edge trigger down
        curControlButton_ = 2;
      prev_B3_state_ = new_B3_state;

      bool new_B4_state = (digital.getInt(4) == 1);
      if (new_B4_state) 
        is_B4_pushed_ = true;
      else
        is_B4_pushed_ = false;
      if (new_B4_state && !prev_B4_state_) // edge trigger down
        curControlButton_ = 3;
      prev_B4_state_ = new_B4_state;

      bool new_B5_state = (digital.getInt(5) == 1);
      if (new_B5_state) 
        is_B5_pushed_ = true;
      else
        is_B5_pushed_ = false;
      if (new_B5_state && !prev_B5_state_) // edge trigger down
        curControlButton_ = 4;
      prev_B5_state_ = new_B5_state;

      bool new_B6_state = (digital.getInt(6) == 1);
      if (new_B6_state) 
        is_B6_pushed_ = true;
      else
        is_B6_pushed_ = false;
      if (new_B6_state && !prev_B6_state_) // edge trigger down
        curControlButton_ = 5;
      prev_B6_state_ = new_B6_state;

      // Note: only care about edge triggers down here
      bool new_mode_button_state = (digital.getInt(7) == 1);
      if (new_mode_button_state && !prev_mode_button_state_)
        ++num_mode_toggles_;
      prev_mode_button_state_ = new_mode_button_state;

      // Check for "Quit"
      if (digital.getInt(8) == 1) {
        has_quit_been_pushed_ = true;
      }
    }
  });
  group_->setFeedbackFrequencyHz(1000); // Don't want to miss button presses!
  return true;
}

void InputManagerMobileIO::printState() const
{
  std::cout << "Rotation (z)" << rot_scale_ * left_horz_raw_ << "\n";
  std::cout << "Rotation (y)" << rot_scale_ * left_vert_raw_ << "\n";

  std::cout << "Translation (x)" << right_vert_raw_ << "\n";
  std::cout << "Translation (y)" << right_horz_raw_ << "\n";
  
  std::cout << "The Robot move? " << shouldMove() << "\n";

  if(is_B1_pushed_)
    std::cout << "Button1 pushed\n";
  if(is_B2_pushed_)
    std::cout << "Button2 pushed\n";
  if(is_B3_pushed_)
    std::cout << "Button3 pushed\n";
  if(is_B4_pushed_)
    std::cout << "Button4 pushed\n";
  if(is_B5_pushed_)
    std::cout << "Button5 pushed\n";
  if(is_B6_pushed_)
    std::cout << "Button6 pushed\n";
  
  std::cout << "current control leg: "<< curControlButton_ <<"\n";

  std::cout << "Height " << xyz_scale_ * getVerticalVelocity() << "\n";
  std::cout << "StepHeight " << getSlide4() << "\n";

  std::cout << "quit state: " << has_quit_been_pushed_ << "\n";
  std::cout << "number of mode changes: " << num_mode_toggles_ << "\n";
}

double InputManagerMobileIO::getRightHorzRaw() const
{
  if (isConnected())
  {
    return right_horz_raw_;
  }
  else
  {
    return 0.0f;
  } 
}
double InputManagerMobileIO::getRightVertRaw() const
{
  if (isConnected())
  {
    return right_vert_raw_;
  }
  else
  {
    return 0.0f;
  } 
}
double InputManagerMobileIO::getLeftHorzRaw() const
{
  if (isConnected())
  {
    return left_horz_raw_;
  }
  else
  {
    return 0.0f;
  } 
}
double InputManagerMobileIO::getLeftVertRaw() const
{
  if (isConnected())
  {
    return left_vert_raw_;
  }
  else
  {
    return 0.0f;
  } 
}
double InputManagerMobileIO::getSlide4() const
{
  if (isConnected())
  {
    const float slider_dead_zone_ = 0.25; 
    if (std::abs(slider_4_raw_) < slider_dead_zone_)
      return 0;
    if (slider_4_raw_ > 0)
      return (slider_4_raw_ - slider_dead_zone_) / (1 - slider_dead_zone_);
    return (slider_4_raw_ + slider_dead_zone_) / (1 - slider_dead_zone_);
  }
  else
  {
    return 0.0f;
  } 
}
double InputManagerMobileIO::getModeNum() const
{
  if (isConnected())
    return num_mode_toggles_;
  else
    return 0.0f;
  
}

bool InputManagerMobileIO::shouldMove() const
{
  if (isConnected())
  {
    return (getRightHorzRaw()!=0) || (getRightVertRaw()!=0);
  }
  else
  {
    return false;
  }
}

int InputManagerMobileIO::getcurControlButton() const
{
  if (isConnected())
  {
    return curControlButton_;
  }
  else
  {
    return 0;
  }
}

void InputManagerMobileIO::resetCurControlButton()
{
  if (isConnected())
  {
    curControlButton_ = 0;
  }
}

Eigen::Vector3f InputManagerMobileIO::getTranslationVelocityCmd() const
{
  Eigen::Vector3f translation_velocity_cmd;
  if (isConnected())
  {
    translation_velocity_cmd <<
      -xyz_scale_ * right_vert_raw_,
      xyz_scale_ * right_horz_raw_,
      xyz_scale_ * getVerticalVelocity();
  }
  else
  {
    translation_velocity_cmd.setZero();
  }
  return translation_velocity_cmd;
}

Eigen::Vector3f InputManagerMobileIO::getRotationVelocityCmd() const
{
  Eigen::Vector3f rotation_velocity_cmd;
  if (isConnected())
  {
    rotation_velocity_cmd <<
      0,
      -rot_scale_ * left_vert_raw_,
      rot_scale_ * left_horz_raw_;
  }
  else
  {
    rotation_velocity_cmd.setZero();
  }
  return rotation_velocity_cmd;
}

Eigen::VectorXd InputManagerMobileIO::getDirection() const {
  Eigen::VectorXd direction(3);
  if (isConnected())
  {
    direction << right_vert_raw_, right_horz_raw_, 0;
  }
  else
  {
    direction.setZero();
  }
  return direction;
}

bool InputManagerMobileIO::getQuitButtonPushed() const
{
  return has_quit_been_pushed_;
}

size_t InputManagerMobileIO::getAndResetModeToggleCount()
{
  return num_mode_toggles_.exchange(0);
}
  
float InputManagerMobileIO::getVerticalVelocity() const
{
  // Slider dead zone: 0.25
  const float slider_dead_zone_ = 0.25; 
  if (std::abs(slider_1_raw_) < slider_dead_zone_)
    return 0;
  // Scale from [dead_zone, 1] to [0, 1] (and same for [-1, -dead_zone]
  if (slider_1_raw_ > 0)
    return (slider_1_raw_ - slider_dead_zone_) / (1 - slider_dead_zone_);
  return (slider_1_raw_ + slider_dead_zone_) / (1 - slider_dead_zone_);
}

} // namespace input
} // namespace hebi
