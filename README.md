# Checkpoint 8 


    git clone https://github.com/MohammadRobot/barista_robot_description.git



Buiild and source the package

Execute in Terminal #1
	cd ~/ros2_ws && colcon build && source install/setup.bash

    ros2 launch barista_robot_description barista_two_robots.launch.py

 Execute in Terminal #2

    ros2 run robot_chase robot_chase

Execute in Terminal #3

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel