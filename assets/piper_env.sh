# Loaded by login and interactive Bash shells, including after user switching.
# Sourcing these files does not start ROS nodes or access CAN/cameras.
if [ -n "${BASH_VERSION:-}" ]; then
    source /opt/ros/noetic/setup.bash
    source /piper_shell.sh
    if [ -f "${HOME}/double_PiPER/devel/setup.bash" ]; then
        source "${HOME}/double_PiPER/devel/setup.bash"
    fi
fi
