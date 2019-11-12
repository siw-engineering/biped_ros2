sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'


sudo apt update
sudo apt install ros-dashing-desktop

sudo apt install python3-argcomplete

echo "alias rosdashing='source /opt/ros/dashing/setup.bash'" >> ~/.bashrc



sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-vcstool \
  wget



sudo apt install gazebo9 libgazebo9-dev
sudo apt install libyaml-cpp-dev ros-dashing-python-qt-binding ros-dashing-gazebo*
echo "source /usr/share/gazebo-9/setup.sh" >> ~/.bashrc

mkdir src
vcs import src < main.repos

echo "alias rosbiped='source ~/biped_ros2/install/setup.bash'" >> ~/.bashrc


