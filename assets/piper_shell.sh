#!/usr/bin/env bash
# Sourced by the container shell. Only set_sdk builds code; sorc_devel loads
# the existing environment. Neither launches ROS nodes or activates CAN.
sorc_devel() {
    local piper_workspace="${HOME}/double_PiPER"
    if [ ! -f /opt/ros/noetic/setup.bash ]; then
        echo "ROS Noetic setup not found: /opt/ros/noetic/setup.bash" >&2
        return 1
    fi
    if [ ! -f "${piper_workspace}/devel/setup.bash" ]; then
        echo "Workspace setup not found: ${piper_workspace}/devel/setup.bash. Run set_sdk first." >&2
        return 1
    fi
    source /opt/ros/noetic/setup.bash || return
    cd "${piper_workspace}" || return
    source "${piper_workspace}/devel/setup.bash"
}

_piper_build_workspace() {
    source /opt/ros/noetic/setup.bash || return
    cd "${HOME}/double_PiPER" || return
    catkin_make || return
    sorc_devel
}

set_sdk() {
    local piper_sdk_source="${HOME}/piper_sdk"
    # Preserve the existing setup command's SDK branch; do not upgrade an
    # existing checkout or substitute a different SDK during cleanup.
    if [ ! -d "${piper_sdk_source}" ] && [ -d /opt/piper_sdk ]; then
        python3 - <<'PYSDK'
from piper_sdk import Piper, C_PiperInterface, C_PiperForwardKinematics
from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager
PYSDK
        if [ "$?" -ne 0 ]; then
            return 1
        fi
        # The image already contains the verified SDK. Avoid reinstalling a
        # different PyPI SDK or cloning another copy just to build ROS.
        _piper_build_workspace
        return
    fi
    if [ ! -d "${piper_sdk_source}" ]; then
        git clone -b 1_0_0_beta https://github.com/agilexrobotics/piper_sdk.git \
            "${piper_sdk_source}" || return
    fi
    python3 -m pip install "${piper_sdk_source}" || return
    _piper_build_workspace
}
