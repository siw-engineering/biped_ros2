# Table of Contents
1. [Simple Installation](#simple-installation)
2. [Manual Installation](#manual-installation)
    - [Normal](#normal)
    - [Conda](#conda)
3. [Running](#running)
4. [Updating](#updating)
5. [Docker](#docker)

# Simple Installation
1 .  To install ros2 and other dependencies.
```
git clone https://github.com/siw-engineering/biped_ros2 ~/biped_ros2
cd ~/biped_ros2
bash biped_ros2_install.sh
```
Note: Do not run step 2 if you are using a conda environment, see [conda](#conda) section

2 . Once install is finished, close the current terminal and open a new one
```bash
cd ~/biped_ros2
rosdashing
colcon build --symlink-install
# The --symlink-install is such that instead of creating new files in the install folder, it just points to the files in the source folder, i.e. no copied files, easier to iterate changes
```
# Manual Installation
## Normal
1. Install ros2 dashing following the instructions [here](https://index.ros.org//doc/ros2/Installation/Dashing/Linux-Install-Debians/)

2. Install ros2 development tools following the section named **Install development tools and ROS tools** listed [here](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/). The commands have also been copy pasted here for reference (with cyclone DDS dependencies removed).
```bash
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget
```

3. Install Opensplice DDS
`sudo apt install ros-dashing-rmw-opensplice-cpp`

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
```bash
cd ~/biped_ros2
rosdashing
colcon build  --symlink-install
# The --symlink-install is such that instead of creating new files in the install folder, it just points to the files in the source folder, i.e. no copied files, easier to iterate changes
```

## Conda
1. Follow the normal installation until step 6 (step 6 done), or step 1 done in simple installation
2. Create a new conda environment with python 3.6 (skip to 3 if already exists)
```bash
conda create -n <your_env_name> python=3.6
```
3. Install relevant dependencies
```bash
# Check if we are in a conda environment
which pip # should return something like /home/<user>/anaconda3/envs/<your_env_name>/bin/pip

# Check python version
python --version # If not 3.6.x, create new environment

pip install empy==3.3.2
pip install catkin-pkg
pip install lark-parser==0.7.2
```

4. Build
```bash
# Make sure conda env is activated when doing this
cd ~/biped_ros2
rosdashing
colcon build --symlink-install
```

Note: most likely there will be CMake warning messages like this for libssl and libcrypto

`Cannot generate a safe runtime search path for target`

For now it doesn't impact anything and there is no known solution to us on this issue, so it is ignored.
If you can help fix this issue please submit a PR or raise an issue to discuss. 
Related discussions [1](https://github.com/pism/pism/issues/356), 
[2](https://github.com/introlab/rtabmap_ros/issues/131)

5. Run. Follow normal [run](#running) instructions but with conda environment activated

# Running
1. Open new terminal and run `rosdashing` and then `rosbiped`
2. Run `ros2 launch arm_simulation debug_launch_all.launch.py` to run the RRR robotic arm
3. Alternatively, run `ros2 run openai_ros2 robot_arm_random_continuous` to run the arm with random inputs


# Updating
1. Switch to a version of choice (preferably latest) 

```bash
cd ~/biped_ros2
git fetch --tags
git checkout <version_number> # version_number is something like 0.6
```

2. Run `vcs import --recursive src < main.repos` again to update the main repositories
3. Build the project again (`colcon build`)
4. Run

# OpenAI Gym Integration
Refer to [here](https://github.com/siw-engineering/openai_ros2)

# Notes
The biped stuff are not tested from version 0.3 onwards. Might not work properly. 

# Docker

## Status
Currently only the spinningup container is available through CD.
The current implementation requires a GUI, and as such X11 forwarding is necessary to run the containers.

stable-baselines container coming soon.

Headless containers coming soon.

Combined container with option to choose any is still under consideration.

## Prerequisites
1. Running on an X11 system such as Ubuntu 18.04 using xorg
2. [nvidia-docker](https://github.com/NVIDIA/nvidia-docker)

## Running

Run the following command in a terminal.
```bash
xhost +local: && \
docker run --gpus all --privileged --rm \
  -u=1099 \
  -e DISPLAY=$DISPLAY \
  -e USER=defaultuser \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/home/$USER/:/home/$USER/" \
  --name=spinningup_container \
  pohzhiee/docker_sims:spinningup \
  python -m spinup.run td3 --env LobotArmContinuous-v2 --exp_name some_experiment
```

## Notes
### Not enough privilege for `docker run`
  - Run it with sudo, however this is not tested and as such the behaviour is not known

### Transferring trained model to host computer. 
1. If the training is not yet started, you can map the data output directory to your host through adding another -v flag. 
For example, if the data in the container is being saved to 
`/home/defaultuser/spinningup/data/some_experiment/some_experiment_s0/`, 
add 
`-v /home/$USER/host_data_dir:/home/defaultuser/spinningup/data/some_experiment/some_experiment_s0/` as one of the run arguments


2. If the training has already started and no volumes other than X11 and home directories are mapped
  - Run `docker exec -it spinningup_container /bin/bash`
  - Once inside the docker container, run `cp` command to copy from `/home/defaultuser/<some_path_to_data>` to `/home/<your_username>/<some_folder>`
  - Example: `sudo cp -r /home/defaultuser/spinningup/data/some_experiment/some_experiment_s0 /home/pohzhiee/advasdkvn/`
  - Run `exit` to exit the container shell
