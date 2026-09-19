FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        locales curl wget gnupg lsb-release ca-certificates \
        software-properties-common git sudo python3-pip python3-dev \
        cmake build-essential ccache gawk pkg-config \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        libgz-sim8-dev \
        libgz-transport13-dev \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1-mesa-glx libgl1-mesa-dri libegl1 libegl-mesa0 libglu1-mesa libosmesa6 \
        mesa-utils x11-utils x11-apps \
        libxcb-xinerama0 libxkbcommon-x11-0 \
    && mkdir -p /usr/lib/dri \
    && ln -sf /usr/lib/x86_64-linux-gnu/dri/* /usr/lib/dri/ \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gzharmonic \
        ros-humble-ros-gzharmonic-bridge \
        ros-humble-ros-gzharmonic-interfaces \
        ros-humble-ros-gzharmonic-sim \
    && rm -rf /var/lib/apt/lists/*

#RUN apt-get update && apt-get install -y --no-install-recommends \
 #       ros-humble-mavros \
  #      ros-humble-mavros-msgs \
   #     ros-humble-geographic-msgs \
    #    geographiclib-tools \
     #   python3-opencv \
      #  python3-wxgtk4.0 \
#        libgtk-3-dev \
 #      libnotify-dev \
  #      libsdl2-dev \
  #  && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh \
  #  && pip3 install --no-cache-dir \
   #     pymavlink \
    #    MAVProxy \
     #   dronecan \
      #  pexpect \
       # future \
        #transforms3d \
      #  matplotlib \
     #   scipy \
    #&& pip3 install --no-cache-dir \
   #     -f https://extras.wxpython.org/wxPython4/extras/linux/gtk3/ubuntu-22.04 \
  #     wxPython \
 #   && rm -rf /var/lib/apt/lists/*
#
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-slam-toolbox \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-tf-transformations \
        ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000
ARG RENDER_GID=992
ARG VIDEO_GID=44

RUN (groupadd --gid $USER_GID $USERNAME 2>/dev/null || true) && \
    (groupadd --gid $RENDER_GID render 2>/dev/null || groupmod -g $RENDER_GID render) && \
    (groupadd --gid $VIDEO_GID video 2>/dev/null || groupmod -g $VIDEO_GID video) && \
    (useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME 2>/dev/null || true) && \
    usermod -aG sudo,video,render $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo "$USERNAME:$USERNAME" | chpasswd

WORKDIR /workspace
ENV USER=$USERNAME
ENV GZ_VERSION=harmonic
ENV XDG_RUNTIME_DIR=/tmp/runtime-$USERNAME

# Preset Gazebo search paths for models, worlds, and plugins
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/ardupilot_gazebo/build
ENV GZ_SIM_RESOURCE_PATH=/workspace/SIM/Models:/workspace/SIM/Worlds:/workspace/ardupilot_gazebo/models:/workspace/ardupilot_gazebo/worlds

# ArduPilot autotest and local binary paths
ENV PATH=/home/$USERNAME/.local/bin:/workspace/ardupilot/Tools/autotest:$PATH

# Create runtime directory with proper ownership
RUN mkdir -p /tmp/runtime-$USERNAME && chmod 700 /tmp/runtime-$USERNAME && chown -R $USERNAME:$USERNAME /tmp/runtime-$USERNAME
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME /workspace

# Switch to developer user
USER $USERNAME

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/$USERNAME/.bashrc && \
    echo "export PATH=/home/$USERNAME/.local/bin:/workspace/ardupilot/Tools/autotest:\$PATH" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f /workspace/ardupilot/Tools/completion/completion.bash ]; then source /workspace/ardupilot/Tools/completion/completion.bash; fi" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f /home/$USERNAME/.ardupilot_env ]; then source /home/$USERNAME/.ardupilot_env; fi" >> /home/$USERNAME/.bashrc
WORKDIR /workspace
CMD ["/bin/bash"]
