FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# Setup user configuration
ARG USER_UID
ARG USER_GID
ARG USERNAME
ARG LIBRS_VERSION
# Make sure that we have a version number of librealsense as argument
RUN test -n "$LIBRS_VERSION"
RUN test -n "$USERNAME"

# Builder dependencies installation
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \    
    curl \
    python3 \
    python3-dev \
    ca-certificates \
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Download sources
WORKDIR /usr/src
RUN curl https://codeload.github.com/realsenseai/librealsense/tar.gz/refs/tags/v$LIBRS_VERSION -o librealsense.tar.gz 
RUN tar -zxf librealsense.tar.gz \
    && rm librealsense.tar.gz 
RUN ln -s /usr/src/librealsense-$LIBRS_VERSION /usr/src/librealsense

# Build and install
RUN cd /usr/src/librealsense \
    && mkdir build && cd build \
    && cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ../ \
    && make -j$(($(nproc)-1)) all \
    && make install 

# Install pip, moveit2
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
    && apt-get install -y ros-humble-moveit ros-humble-tf-transformations \
    && sudo apt-get clean && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip3 install --upgrade pip

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
COPY kortex_api-2.7.0.post5-py3-none-any.whl kortex_api-2.7.0.post5-py3-none-any.whl
RUN pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu129 \
    && pip3 install numpy==1.26.4 \
    # && pip3 install -U xformers --index-url https://download.pytorch.org/whl/cu129 \
    && pip3 install -r requirements.txt \
    && pip3 install kortex_api-2.7.0.post5-py3-none-any.whl
RUN pip3 install --upgrade transforms3d

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "/bin/bash" ]
