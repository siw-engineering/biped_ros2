```bash
wget https://raw.githubusercontent.com/siw-engineering/biped_ros2/dashing/custom_ros2_control.repos
wget https://raw.githubusercontent.com/siw-engineering/biped_ros2/dashing/lobot.repos
vcs import src < custom_ros2_control.repos
vcs import src < lobot.repos
```