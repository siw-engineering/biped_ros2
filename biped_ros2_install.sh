sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt-get update && sudo apt-get install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'


sudo apt-get update
sudo apt-get install -y ros-dashing-desktop

sudo apt-get install-y python3-argcomplete

echo "alias rosdashing='source /opt/ros/dashing/setup.bash'" >> ~/.bashrc



sudo apt-get update && sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-vcstool \
  wget

# Install Opensplice DDS
sudo apt-get install -y ros-dashing-rmw-opensplice-cpp
# Install gazebo
sudo apt-get install -y gazebo9 libgazebo9-dev
# Install other dependencies (yaml, gazebo_ros_pkgs, etc.)
sudo apt-get install -y libyaml-cpp-dev ros-dashing-python-qt-binding ros-dashing-gazebo*
echo "source /usr/share/gazebo-9/setup.sh" >> ~/.bashrc

mkdir src
vcs import src < main.repos

echo "alias rosbiped='source ~/biped_ros2/install/setup.bash'" >> ~/.bashrc


