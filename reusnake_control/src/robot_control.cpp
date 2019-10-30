// stl
#include <iostream>
#include <set>
#include <chrono>
#include <thread>
#include <atomic>

#include "input/input_manager_mobile_io.hpp"
#include "util/util.hpp"
#include "robot/robot_base.hpp"
#include "robot/reusnake.hpp"
#include "gait/control_state_def.hpp"  // definition of control states

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_control");
  ros::NodeHandle nh;
  /* parse input arguments */
  bool do_visualize{};
  bool is_dummy{};
  bool is_quiet{};
  int robot_type{};
  if (!robot::parse_args(argc, argv, do_visualize, is_dummy, robot_type, is_quiet)) {
    return 1;
  }
  
  /* create instance according to robot type and loaded parameters */
  std::unique_ptr<robot::RobotBase> robot_base;
  try {
    if (robot_type == 0) {        // reusnake 
      robot_base = robot::Reusnake::create(nh);
    }  else {
      throw std::invalid_argument("Robot type not supported.");
    }

  } catch(const std::invalid_argument& e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch(const std::ios_base::failure& e) {
    std::cout << e.what() << ". Check if xml file available in build dir.\n";
    return -1;
  }

  // a publisher publish state of the controller, so the robot_estimation node can use this state to initialize
  // or change estimator status
  ros::Publisher ctrl_state_pub = nh.advertise<std_msgs::Int32>("/control_state", 1000);

  std_msgs::Int32 ctrl_state_msg;
  std_msgs::Float32 phase_msg;

  /* connect to input device */
  std::unique_ptr<hebi::input::InputManager> input(new hebi::input::InputManagerMobileIO());
  

  // by pass input logic for towr traj
  // Retry a "reset" multiple times! Wait for this in a loop.
  int connect_count = 0;
  while (!input->isConnected() && ros::ok()) {
    std::cout << "Could not find control input device. Retry." << std::endl;
    connect_count ++;
    static_cast<hebi::input::InputManagerMobileIO*>(input.get())->reset();
    // add this to bypass input device checking logic only for debug
    // if (connect_count > 3) {
    //   break;
    // }
  } 
  if (connect_count <= 3) {
    std::cout << "Input device connected -- starting control thread.\n";
    std::cout << "Input device connected -- main gait mode will be wave gait.\n";
  } else {
    std::cout << "Input device not connected -- starting control thread.\n";
    std::cout << "Input device not connected -- main gait mode will be trajectory following mode.\n";
  }

  /* start control thread */
  std::atomic<bool> control_execute;
  control_execute.store(true, std::memory_order_release);
  std::cout << "Enter Main Loop -- Another thread.\n";
  std::thread control_thread([&]() {  // start of the control thread
    // prepare variables to moni tor time and control the while loop
    long interval_ms = 20;    //  10ms - 100Hz control frequency
    auto start = std::chrono::steady_clock::now();     // time instance when the control thread starts
    auto prev = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();  // bool res = app.exec();
    std::chrono::duration<double> dt = std::chrono::microseconds(0);

    // setup variables for state to track time 
    auto state_enter_time = std::chrono::steady_clock::now(); 
    auto state_curr_time = std::chrono::steady_clock::now(); 
    std::chrono::duration<double> state_run_time = std::chrono::seconds(0);

    // initial values for control state, used to switch between states
    ctrl_state_type curr_ctrl_state = REUSNAKE_SIDEWINDING;
    bool first_time_enter = true;

    // state transition parameters
    Eigen::Vector3f translation_velocity_cmd;
    translation_velocity_cmd.setZero();
    Eigen::Vector3f rotation_velocity_cmd;
    rotation_velocity_cmd.setZero();
    double gait_t = 0;


    // this is the main control loop
    while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
      // sleep for interval_ms
      now = std::chrono::steady_clock::now();
      dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - prev);
      auto need_to_wait = std::max(0, (int)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::microseconds(interval_ms*1000)-dt).count());
      std::this_thread::sleep_for(std::chrono::microseconds(need_to_wait));

      // Get dt (in seconds)
      now = std::chrono::steady_clock::now();
      dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - prev);
      prev = now;

      // Time since the control thread starts (in seconds)
      std::chrono::duration<double> elapsed(now - start);

      // if (!is_quiet) std::cout << "CTRL THREAD: Time since start: " << elapsed.count() << " | delta time is:" << dt.count() << std::endl;

      // get control input
      // Get joystick update, and update any relevant variables.
      input->update();
      // input -> printState();
      if (input->getQuitButtonPushed()) {
        // exit qt program
        // app.exit();

        // break the control thread and spin thread
        control_execute.store(false, std::memory_order_release);
      }
      
      // publish state msg
      ctrl_state_msg.data = curr_ctrl_state;
      ctrl_state_pub.publish(ctrl_state_msg);
      /* control state machine governs the behaviour planning of the robot*/
      switch (curr_ctrl_state)
      {
        case REUSNAKE_SIDEWINDING:
        {
          // if (!is_quiet) std::cout << "state REUSNAKE_SIDEWINDING" <<std::endl;
          state_curr_time = std::chrono::steady_clock::now();
          state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);


          Eigen::VectorXd direction = input -> getDirection();
          direction(1) = -direction(1);// because for robot, the x positive is forward and y positive is leftward
          double yawVel = input -> getLeftHorzRaw();

          if (direction(0) > 0.1) {
            gait_t += 0.004;
          } else if (direction(0) < -0.1) {
            gait_t -= 0.004;
          }

          // plan start up trajectory
          // robot_base -> planStandUpTraj();  
          robot_base -> gait_sidewinding(gait_t);
          robot_base -> sendCommand();       

          // // transit after plan the trajectories
          // curr_ctrl_state = TITAN6_CTRL_STAND_UP2;
          // state_enter_time = std::chrono::steady_clock::now();
          break;
        }
        // end of different ctrl states
      }

    }

    // end of the control thread
  });  

  // ros spinner
  std::thread spin_thread([&](){
    while(control_execute.load(std::memory_order_acquire) && ros::ok())
      ros::spinOnce();
  });


  // control_execute.store(false, std::memory_order_release);
  control_thread.join();
  spin_thread.join();
  return 0;
}
