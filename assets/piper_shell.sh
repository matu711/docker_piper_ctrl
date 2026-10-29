#!/usr/bin/env bash
# Sourced by the container shell. Building/loading a workspace never launches
# ROS nodes or activates CAN/cameras. Always use Noetic's system Python.
_piper_workspace() {
    # Migrate the legacy value inherited from already-created containers.
    # Other explicit workspaces remain available for isolated build tests.
    case "${PIPER_WORKSPACE:-}" in
        ''|"${HOME}/double_PiPER")
            printf '%s\n' "${HOME}/docker_piper_ctrl/double_PiPER" ;;
        *) printf '%s\n' "${PIPER_WORKSPACE}" ;;
    esac
}

sorc_devel() {
    local piper_workspace
    piper_workspace=$(_piper_workspace)
    if [ ! -f /opt/ros/noetic/setup.bash ]; then
        echo "ROS Noetic setup not found: /opt/ros/noetic/setup.bash" >&2
        return 1
    fi
    if [ ! -f "${piper_workspace}/devel/setup.bash" ]; then
        echo "Workspace setup not found: ${piper_workspace}/devel/setup.bash. Run set_sdk first." >&2
        return 1
    fi
    # Do not execute a stale catkin helper whose shebang points at host pyenv.
    if [ "$(head -n 1 "${piper_workspace}/devel/_setup_util.py")" != '#!/usr/bin/python3' ]; then
        echo "catkin setup does not use /usr/bin/python3. Run set_sdk to regenerate it; existing source/data are preserved." >&2
        return 1
    fi
    source /opt/ros/noetic/setup.bash || return
    cd "${piper_workspace}" || return
    source "${piper_workspace}/devel/setup.bash" || return
    # Reject an old overlay instead of silently launching a different package.
    local piper_package
    piper_package=$(rospack find piper) || return
    if [ "${piper_package}" != "${piper_workspace}/src/piper" ]; then
        echo "Wrong piper package: ${piper_package}; expected ${piper_workspace}/src/piper. Run set_sdk." >&2
        return 1
    fi
}

_piper_build_workspace() {
    local piper_workspace piper_cache_source piper_cache_build piper_backup
    piper_workspace=$(_piper_workspace)
    source /opt/ros/noetic/setup.bash || return
    cd "${piper_workspace}" || return
    if [ -f build/CMakeCache.txt ]; then
        piper_cache_source=$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' build/CMakeCache.txt)
        piper_cache_build=$(sed -n 's/^CMAKE_CACHEFILE_DIR:INTERNAL=//p' build/CMakeCache.txt)
        # CMake embeds textual absolute paths, even when two bind-mount aliases
        # refer to the same inode. Reconfigure in the one canonical workspace.
        if { [ -n "${piper_cache_source}" ] && [ "${piper_cache_source}" != "${piper_workspace}/src" ]; } ||
           { [ -n "${piper_cache_build}" ] && [ "${piper_cache_build}" != "${piper_workspace}/build" ]; }; then
            # A relocated CMake cache cannot be reused. Move only identified
            # generated directories, retaining a complete, unique backup.
            if [ -e devel ] && [ ! -f devel/.catkin ]; then
                echo 'devel is not an identified catkin output; inspect it before rebuilding.' >&2
                return 1
            fi
            piper_backup=$(mktemp -d "${piper_workspace}/.piper-generated-backup.XXXXXXXX") || return
            mv build "${piper_backup}/build" || return
            if [ -e devel ]; then
                mv devel "${piper_backup}/devel" || return
            fi
            echo "Relocated catkin outputs preserved in ${piper_backup}"
        fi
    fi
    # catkin_make may otherwise run make's cache checks before applying -D.
    # Force configuration so an old missing interpreter cannot run first.
    catkin_make --force-cmake -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@" || return
    sorc_devel
}

set_sdk() {
    local piper_sdk_source="${HOME}/piper_sdk"
    # Preserve the established SDK branch; never upgrade an existing checkout.
    if [ ! -d "${piper_sdk_source}" ] && [ -d /opt/piper_sdk ]; then
        /usr/bin/python3 - <<'PYSDK'
from piper_sdk import Piper, C_PiperInterface, C_PiperForwardKinematics
from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager
PYSDK
        if [ "$?" -ne 0 ]; then
            return 1
        fi
        _piper_build_workspace "$@"
        return
    fi
    if [ ! -d "${piper_sdk_source}" ]; then
        git clone -b 1_0_0_beta https://github.com/agilexrobotics/piper_sdk.git \
            "${piper_sdk_source}" || return
    fi
    /usr/bin/python3 -m pip install "${piper_sdk_source}" || return
    _piper_build_workspace "$@"
}
