#include "reusnake.hpp"

// for calculate inverse of jacobian
#include <Eigen/SVD>
#include <Eigen/QR>


namespace robot {

std::unique_ptr<Reusnake> Reusnake::create(ros::NodeHandle &_nh) {
  hebi::Lookup lookup;
  std::vector<std::string> names;
  std::vector<std::string> family;
  
  std::cout<<"look for Reusnake Robot modules on network"<<std::endl;
  for (int i = 0; i < 14; ++i) {
      names.push_back("RUSnake Module #" + std::to_string(i + 1));
  }
  family = { "RUSNAKE" };  

  long timeout_ms = 4000; // use a 4 second timeout
  // group is created here, it has the same size of the actual numbers of modules on robot
  std::shared_ptr<hebi::Group> group;
  group = lookup.getGroupFromNames(family, names, timeout_ms);
  // group = lookup.getGroupFromFamily("RUSNAKE", timeout_ms);
  if (!group) {
    std::cout << "cannot found Hebi modules on network!" << std::endl;
    return nullptr;
  }
  
  assert(group->setCommandLifetimeMs(400));

  // Log everything!
  std::shared_ptr<hebi::Group> log_group_modules;
  std::shared_ptr<hebi::Group> log_group_io;
  log_group_io = lookup.getGroupFromNames({"HEBI"}, {"Mobile IO"}, timeout_ms);
  log_group_modules = lookup.getGroupFromNames(family, names, timeout_ms);

  std::cout<<"Found Reusnake robot modules. Prepare to control the robot..."<<std::endl;
  return std::unique_ptr<Reusnake>(new Reusnake(group, log_group_io, log_group_modules, _nh));
}

Reusnake::Reusnake(std::shared_ptr<hebi::Group> group, std::shared_ptr<hebi::Group> log_group_input, std::shared_ptr<hebi::Group> log_group_modules, ros::NodeHandle &_nh) {
  
  group_ = group;
  log_group_input_ = log_group_input;
  log_group_modules_ = log_group_modules;
  nh = _nh;

  if (!setGains() && !setGains()){
    std::cout << "\nCould not set controller gains on connected modules -- this could indicate an intermittent network connection with the modules.";
  }
  else {
    std::cout << "\nController gains are successfully set.";
  }

  // necessary sensor data
  // single IMU of base 1
  imu_pub = nh.advertise<sensor_msgs::Imu>("reusnake_state/fbk/imu1", 2000);
  imu_msg.header.frame_id = "link0";
 
  // start to read feedback
  if (group_) {
      group_->setFeedbackFrequencyHz(104);
      hebi::GroupFeedback fbk(group_->size());
      // TODO: ensure feedback? Get before entering function?
      group_->sendFeedbackRequest();  // Potential bug in 1.0.0-rc3 (C lib -rc4) means I have to call this twice, depending on OS.
      group_->getNextFeedback(fbk);
      group_->sendFeedbackRequest();    
  }

  // I think this should be put somwhere in robot base, but it must be put here
  init();

  // Start a background feedback handler
  // must be started after all other parts are instantiated 
  if (group_) {
    group->addFeedbackHandler([this] (const hebi::GroupFeedback& fbk) {

      std::lock_guard<std::mutex> guard(hebi_fbk_lock_);
      curr_fbk_time_ = std::chrono::steady_clock::now();
      fbk_dt_ = curr_fbk_time_ - last_fbk_time_;
      last_fbk_time_ = curr_fbk_time_;


      assert(fbk.size() == 14);


      // std::cout << fbk.size() << " fbk angles: ";
      // for (int i = 0; i < numModules; ++i) {
      //   std::cout << fbk[i].actuator().position().get() << "\t";
      // }
      // std::cout << std::endl;

      // read IMU from base 1
      // HEBI Quaternion
      auto mod_orientation = fbk[0]   // 0  3  6 9 12 15
        .imu().orientation().get();

      hebi::Vector3f acc = fbk[0].imu().accelerometer().get();
      hebi::Vector3f gyro = fbk[0].imu().gyro().get();

      // publish IMU from base 1
      
      imu_msg.orientation.x = mod_orientation.getX();
      imu_msg.orientation.y = mod_orientation.getY();
      imu_msg.orientation.z = mod_orientation.getZ();
      imu_msg.orientation.w = mod_orientation.getW();     

      imu_msg.angular_velocity.x = gyro.getX();
      imu_msg.angular_velocity.y = gyro.getY();
      imu_msg.angular_velocity.z = gyro.getZ();

      imu_msg.linear_acceleration.x = acc.getX();
      imu_msg.linear_acceleration.y = acc.getY();
      imu_msg.linear_acceleration.z = acc.getZ();

      // do not publish ROS msg here, put them in a loop
      ros::Time curr_ros_time = ros::Time::now();         
      imu_msg.header.stamp = curr_ros_time;


      imu_pub.publish(imu_msg);

      // end of group feedback handler
    });  


  }
}

Reusnake::~Reusnake () {
 
}

}    