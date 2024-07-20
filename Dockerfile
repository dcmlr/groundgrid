# Use an official ROS1 environment as a parent image
FROM ros:noetic-robot

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Set a non-interactive shell to avoid stuck prompts during build
ARG DEBIAN_FRONTEND=noninteractive

# Update package lists and install only necessary packages
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  sudo \
  lsb-release \
  gnupg2 \
  net-tools \
  clang-format \
  clangd \
  gdb \
  libpcl-dev \
  libopencv-dev \
  python3-pip \
  python3-catkin-tools \
  python3-rospkg \
  python3-rospy \
  ros-noetic-tf2-ros \
  ros-noetic-sensor-msgs \
  ros-noetic-catkin \
  ros-noetic-pcl-ros \
  ros-noetic-grid-map \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Set Python 3 as the default Python version
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# Copy requirements.txt and install Python packages using pip
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

# Add a user for the development environment
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create user with specified home directory
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID --create-home --home-dir /home/$USERNAME $USERNAME \
  && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

# Ensure .bashrc exists and source it in the entrypoint script
RUN touch /home/$USERNAME/.bashrc

# Set the working directory and change ownership
WORKDIR /home/$USERNAME/workspace
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/workspace

# Setup ROS1 environment in the bashrc for interactive bash shells
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc

# Switch to the non-root user
USER $USERNAME

# Command to run on container start
CMD ["/bin/bash"]
