# Installation
1. Install ros2 dashing following the instructions [here](https://index.ros.org//doc/ros2/Installation/Dashing/Linux-Install-Debians/)

2. Install ros2 development tools following the section named **Install development tools and ROS tools** listed [here](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/)

3. Install Gazebo
`sudo apt install gazebo9 libgazebo9-dev`

4. Install other dependencies 
`sudo apt install libyaml-cpp-dev ros-dashing-gazebo*`
5. Download the relevant source code
```bash
git clone https://github.com/siw-engineering/biped_ros2 ~/biped_ros2
cd ~/biped_ros2
mkdir src
vcs import src < custom_ros2_control.repos
vcs import src < lobot.repos
vcs import src < debugging_tools.repos
```
Note: if `vcs not found` error occured, try installing the development tools from the step 2. If it still doesn't work, try `pip3 install vcstool`.

6. Add the following lines to ~/.bashrc
```bash
alias rosbiped='source ~/biped_ros2/install/setup.bash'
alias rosdashing='source /opt/ros/dashing/setup.bash'
source /usr/share/gazebo-9/setup.sh
```
7. Build the project. Run the command `rosdashing` and then `colcon build` in the **~/biped_ros2** directory

# Running
1. Open terminal and run `rosbiped`
2. Run `ros2 launch lobot_control_main launch_all_debug.launch.py

# OpenAI Gym
Refer to [here](https://github.com/pohzhiee/biped_gym)

# Notes
1. There is currently a bug where the controllers and the robot plugins do not discover each other quickly enough. This causes the robot to seem like it is not controlled when it is spawned. You might need to wait for some time before they discover each other and the robot move into a fixed position. This bug is partially beyond our control as some of it has to do with the discovery mechanism of the ROS publisher/subscribers.`
