FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# Register Intel RealSense public key and repository
RUN sudo mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
        sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/librealsense.list && \
    sudo apt-get update && \
    sudo apt-get install -y \
        librealsense2-utils \
        librealsense2-dev \
        librealsense2-dbg && \
    sudo apt-get clean && rm -rf /var/lib/apt/lists/*

# Install pip, moveit2
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
    && apt-get install -y ros-humble-moveit \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip3 install --upgrade pip

# Setup user configuration
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=zone

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc
    
USER $USERNAME

WORKDIR /home/$USERNAME

# Install torch, torchvision and other ML packages
# NOTE: Things are installed under the user, not sure what'd be the consequence
# NOTE: Modify this to match cuda version on the host machine
COPY requirements.txt requirements.txt
COPY kortex_api-3.3.0.2-py3-none-any.whl kortex_api-3.3.0.2-py3-none-any.whl
RUN pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu129 \
    && pip3 install numpy==1.26.4 \
    && pip3 install -U xformers --index-url https://download.pytorch.org/whl/cu129 \
    && pip3 install -r requirements.txt \
    && pip3 install kortex_api-3.3.0.2-py3-none-any.whl

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "/bin/bash" ]
