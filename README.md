# spars_rover
SPARS 2 planner-based rover UI Ifor the Sentry Robot built via @cwru_robotics for the U.S. Airforce 

# installation
- cd ~/projects/ros_ws/src
- git clone https://github.com/ShaunHoward/spars_rover.git
- git clone https://github.com/clearpathrobotics/occupancy_grid_utils.git
- install ros indigo (latest turtlebot capable version), make sure to run: echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc and source the .bashrc
- sudo apt-get install libsdformat1 gazebo2 ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs libbullet-dev ros-indigo-tf2-bullet
- restart shell for changes to take effect
- cd ~/projects/ros_ws/src
- rosdep install --from-paths . --ignore-src --rosdistro=indigo

# create map
- roslaunch turtlebot_rrt start_sim_map.launch
- roslaunch turtlebot_teleop keyboard_teleop.launch
- create the map by driving around with keyboard commands
- once map is ready to save: rosrun map_server map_saver -f /path/to/map_file

# run robot simulator and planner with map
- cd ~/projects/ros_ws
- catkin_make
- roslaunch turtleb start_sim_plan.launch

# create new layer for costmap
- http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer
