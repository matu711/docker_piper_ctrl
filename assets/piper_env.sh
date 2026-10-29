# Loaded by login and interactive Bash shells, including after user switching.
# Sourcing these files does not start ROS nodes or access CAN/cameras.
if [ -n "${BASH_VERSION:-}" ]; then
    source /opt/ros/noetic/setup.bash
    source /piper_shell.sh
    export PIPER_WORKSPACE="$(_piper_workspace)"
    if [ -f "$(_piper_workspace)/devel/setup.bash" ]; then
        piper_env_start_directory=$PWD
        sorc_devel
        cd -- "${piper_env_start_directory}"
        unset piper_env_start_directory
    fi
fi
