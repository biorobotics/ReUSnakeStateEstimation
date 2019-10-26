ReUSnakeStateEstimation

The general idea: Use a Matlab script to control snake and read feedback. Send feedback to ROS using Matlab ROS toolbox. Use reusnake_visualize to visualize the snake in Rviz.
Use VINS-fusion to get visual odometry.

If VINS-fusion can work alone, we are happy. If not, we need to modify VINS-fusion to use additional information from snake sensors to improve the state estimation performance. 

### Install 
```shell
mkdir -p snake_ws/src
cd snake_ws/src
catkin_init_workspace
cd ..
catkin_make
cd src
git clone https://github.com/biorobotics/ReUSnakeStateEstimation.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

solve dependency issues arised in compiling VINS

### Basic Usage

1. Make sure your laptop and snake are in the same network. 
2. connect a joystick to your laptop
3. Get into reusnake_matlab and use joy_test_snake.m to drive the snake.
4. You may encounter problems when running joy_test_snake.m because the joystick function of matlab will perform differently on different laptops with different types of joystick (the array "buttons" will have different length). You should be able to easily fix that.
