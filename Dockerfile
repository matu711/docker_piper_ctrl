FROM ubuntu:20.04

ARG USR_NAME=dnp218
ARG USER_UID=1000
ARG USER_GID=1000
# The same revision as the previously verified set_sdk 1_0_0_beta branch.
ARG PIPER_SDK_REF=ad16ba7be8dde7672da5b46fcf6780291317650d
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8 \
    TZ=Asia/Tokyo

# Bootstrap the ROS apt repository; avoid the deprecated keyserver setup.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg locales tzdata && \
    locale-gen en_US.UTF-8 && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main' \
      > /etc/apt/sources.list.d/ros-noetic.list && \
    rm -rf /var/lib/apt/lists/*

# ROS uses Ubuntu's Python 3.8. NumPy/OpenCV/YAML/EmPy come from apt only,
# keeping cv_bridge and catkin on the same interpreter and NumPy ABI.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git gosu sudo \
      can-utils ethtool iproute2 v4l-utils \
      python3-pip python3-setuptools python3-wheel python3-empy \
      python3-numpy python3-opencv python3-yaml python3-rosdep \
      ros-noetic-ros-base ros-noetic-catkin \
      ros-noetic-message-generation ros-noetic-message-runtime \
      ros-noetic-rospy ros-noetic-roslaunch ros-noetic-rosgraph \
      ros-noetic-rosnode ros-noetic-rostopic \
      ros-noetic-cv-bridge ros-noetic-tf \
      ros-noetic-geometry-msgs ros-noetic-sensor-msgs \
      ros-noetic-std-msgs ros-noetic-std-srvs && \
    rm -rf /var/lib/apt/lists/*

# SDK and MCAP stay in /usr/bin/python3's environment. No PyPI OpenCV,
# NumPy, EmPy, or second SDK version is installed over the ROS apt packages.
# SDK 1.0 creates log directories even with file logging off; grant write
# access only to that log directory, keeping the installed code root-owned.
RUN git init /opt/piper_sdk && \
    git -C /opt/piper_sdk remote add origin https://github.com/agilexrobotics/piper_sdk.git && \
    git -C /opt/piper_sdk fetch --depth 1 origin "${PIPER_SDK_REF}" && \
    git -C /opt/piper_sdk checkout --detach FETCH_HEAD && \
    python3 -m pip install --no-cache-dir \
      python-can==4.5.0 mcap==1.6.0 mcap-ros1-support==0.9.0 /opt/piper_sdk && \
    piper_sdk_install=$(python3 -c 'import importlib.util; print(next(iter(importlib.util.find_spec("piper_sdk").submodule_search_locations)))') && \
    mkdir -p "${piper_sdk_install}/log" && \
    chmod 1777 "${piper_sdk_install}/log" && \
    rosdep init

RUN groupadd --gid "${USER_GID}" "${USR_NAME}" && \
    useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USR_NAME}" && \
    mkdir -p "/home/${USR_NAME}/docker_piper_ctrl" && \
    chown "${USR_NAME}:${USR_NAME}" "/home/${USR_NAME}/docker_piper_ctrl"

COPY assets/piper_shell.sh /piper_shell.sh
COPY assets/piper_env.sh /etc/profile.d/piper.sh
COPY assets/entrypoint_setup.sh /entrypoint_setup.sh
RUN chmod 755 /entrypoint_setup.sh && \
    echo '. /etc/profile.d/piper.sh' >> /etc/bash.bashrc

# Restore the interactive terminal only; keep the CPU/ROS-base environment.
RUN apt-get update && apt-get install -y --no-install-recommends terminator && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USR_NAME}/docker_piper_ctrl
ENTRYPOINT ["/entrypoint_setup.sh"]
CMD ["terminator", "--no-dbus", "--command=/bin/bash"]
