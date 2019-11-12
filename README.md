# Installation

1 .  To install ros2 and other dependencies.
```
bash biped_ros2_install.sh
```
2 . Once install is finished, close the current terminal and open a new one
```
cd ~/biped_ros2
rosdashing
colcon build
```
    
## Alternatively

1. Install ros2 dashing following the instructions [here](https://index.ros.org//doc/ros2/Installation/Dashing/Linux-Install-Debians/)

2. Install ros2 development tools following the section named **Install development tools and ROS tools** listed [here](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/)

3. Install Gazebo
`sudo apt install gazebo9 libgazebo9-dev`

4. Install other dependencies 
`sudo apt install libyaml-cpp-dev ros-dashing-python-qt-binding ros-dashing-gazebo*`
5. Download the relevant source code. The optional repositories are mainly for URDF development
as well as the development of binaries of utility libraries.
```bash
git clone https://github.com/siw-engineering/biped_ros2 ~/biped_ros2
cd ~/biped_ros2
mkdir src
vcs import src < main.repos
(optional) vcs import src < optional.repos
```
Note: if `vcs not found` error occured, try installing the development tools from the step 2. If it still doesn't work, try `pip3 install vcstool`.

6. Add the following lines to ~/.bashrc
```bash
alias rosbiped='source ~/biped_ros2/install/setup.bash'
alias rosdashing='source /opt/ros/dashing/setup.bash'
source /usr/share/gazebo-9/setup.sh
```
Note: remember to restart your terminal after changing `~/.bashrc`. Alternatively you can run `source ~/.bashrc` to use the new bashrc config.

7. Build the project. 
```
cd ~/biped_ros2
rosdashing
colcon build
```

# Running (arm)
1. Open new terminal and run `rosdashing` and then `rosbiped`
2. Run `ros2 launch arm_simulation debug_launch_all.launch.py` to run the RRR robotic arm

# Running (biped)
1. Open new terminal and run `rosdashing` and then `rosbiped`
2. Run `ros2 launch lobot_simulation debug_launch_all.launch.py` to run the biped (representing a lobot h3p)


# Updating
1. Checkout and pull the updated branch from remote
2. Run `vcs import src < main.repos` again to update the main repositories
3. Build the project again (`colcon build`)
4. Run

# OpenAI Gym Integration
Refer to [here](https://github.com/siw-engineering/openai_ros2)

# Notes
The biped stuff are not tested from version 0.3 onwards. Might not work properly. 
