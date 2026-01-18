FROM osrf/ros:jazzy-desktop
SHELL ["/bin/bash", "-exo", "pipefail", "-c"]

# set up groundgrid
USER ubuntu
WORKDIR /home/ubuntu
RUN echo 'source /opt/ros/jazzy/setup.bash' | tee -a ~/.bashrc
RUN mkdir -p ros/src/groundgrid
COPY ./ ros/src/groundgrid/
RUN rosdep update

# install dependencies
USER root
WORKDIR /home/ubuntu/ros
RUN chown -R ubuntu:ubuntu src/groundgrid
RUN apt-get update
RUN apt-get install python3-prettytable -y
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro jazzy -y

# build groundgrid
USER ubuntu
WORKDIR /home/ubuntu/ros
RUN source /opt/ros/jazzy/setup.bash && colcon build
RUN echo 'source /home/ubuntu/ros/install/setup.bash' | tee -a ~/.bashrc
WORKDIR /home/ubuntu/ros
ENTRYPOINT [ "/bin/bash", "-l" ]
