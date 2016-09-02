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

# install opencv
- Download the installation script at: https://github.com/milq/scripts-ubuntu-debian/blob/master/install-opencv.sh
- open your terminal and execute: ```bash install-opencv.sh```
- Type your sudo password and you will have installed OpenCV. This operation may take a long time due to the packages to be installed and the compilation process.

# install caltech lane-following software
- Prerequisites
1. OpenCV 2.0 or higher http://sourceforge.net/projects/opencvlibrary/
2. (Optional) Gengetopt http://www.gnu.org/software/gengetopt/
- git clone https://github.com/aranyadan/caltech-lane-detection.git
- cd ~/ros_ws/src/caltech-lane-detection/
- make release
- This will generate LaneDetector32 or LaneDetector64 depending on your system.
- look at caltech-lane-detection folder README.txt for more info on running on caltech dataset and matlab statistics

# create map
- roslaunch spars_rover start_sim_map.launch
- roslaunch spars_rover keyboard_teleop.launch
- create the map by driving around with keyboard commands
- once map is ready to save: rosrun map_server map_saver -f /path/to/map_file

# run robot simulator and planner with map
- cd ~/projects/ros_ws
- catkin_make
- roslaunch spars_rover start_sim_plan.launch
