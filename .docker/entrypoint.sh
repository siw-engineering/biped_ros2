#!/bin/bash
set -e

# setup ros2 environment
source "/home/defaultuser/biped_ros2/install/setup.bash"
source activate conda_env

exec "$@"