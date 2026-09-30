# Source this before using the workspace:  source ~/ros2_ws/env.sh
# Put system python ahead of Homebrew's, otherwise ROS python tools/nodes break.
export PATH=/usr/bin:$PATH
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Gazebo GUI is more reliable via XWayland on GNOME Wayland sessions.
export QT_QPA_PLATFORM=xcb

# From SSH/VS Code remote terminals, show GUIs on the desktop that is logged in on this PC.
# The XWayland auth filename changes every login, hence the glob.
if [ -z "$DISPLAY" ]; then
    export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}
    _xauth=$(ls "$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.* 2>/dev/null | head -n1)
    if [ -n "$_xauth" ]; then
        export DISPLAY=:0
        export XAUTHORITY=$_xauth
    else
        echo "env.sh: no desktop session found (is someone logged in on the PC?); GUIs won't open." >&2
    fi
    unset _xauth
fi
