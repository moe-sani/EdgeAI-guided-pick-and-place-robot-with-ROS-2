# Source this before using the workspace:  source ~/ros2_ws/env.sh
# Put system python ahead of Homebrew's, otherwise ROS python tools/nodes break.
export PATH=/usr/bin:$PATH
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Gazebo GUI is more reliable via XWayland on GNOME Wayland sessions.
export QT_QPA_PLATFORM=xcb
