# turtlebot_rrt
Path planning with the Turtlebot using various RRT algorithms from ompl.

# installation instructions
- cd ~/projects/ros_ws/src
- git clone https://github.com/ShaunHoward/turtlebot_rrt.git
- git clone https://github.com/clearpathrobotics/occupancy_grid_utils.git
- install ros indigo (latest turtlebot capable version), make sure to run: echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc and source the .bashrc
- sudo apt-get install libsdformat1 gazebo2 ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs libbullet-dev
- restart shell for changes to take effect
- cd ~/projects/ros_ws/src
- rosdep install --from-paths . --ignore-src --rosdistro=indigo

# run robot simulator and planner
- cd ~/projects/ros_ws
- catkin_make
- roslaunch turtlebot_rrt start_rrt_benchmark.launch
