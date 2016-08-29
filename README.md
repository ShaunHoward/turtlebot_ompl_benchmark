# turtlebot_ompl_benchmark
Path planning with the Turtlebot using various geometric path planning algorithms from OMPL.

# installation instructions
- cd ~/projects/ros_ws/src
- git clone https://github.com/ShaunHoward/turtlebot_ompl_benchmark.git
- git clone https://github.com/clearpathrobotics/occupancy_grid_utils.git
- install ros indigo (latest turtlebot capable version), make sure to run: echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc and source the .bashrc
- sudo apt-get install libsdformat1 gazebo2 ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs libbullet-dev ros-indigo-tf2-bullet
- restart shell for changes to take effect
- cd ~/projects/ros_ws/src
- rosdep install --from-paths . --ignore-src --rosdistro=indigo

# create map
- roslaunch turtlebot_rrt start_sim_map.launch
- roslaunch turtlebot_teleop keyboard_teleop.launch
- create the map by driving around with keyboard commands
- once map is ready to save: rosrun map_server map_saver -f /path/to/map_file

# run robot simulator and planner with known map
- cd ~/projects/ros_ws
- catkin_make
- roslaunch turtlebot_rrt start_sim_plan.launch 
