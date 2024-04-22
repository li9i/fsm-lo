FROM osrf/ros:kinetic-desktop
MAINTAINER Alexandros Philotheou alefilot@auth.gr

RUN apt-get update
RUN apt-get install -y sudo apt-utils build-essential g++ git libfftw3-dev libcgal-dev=4.7-4 libcgal-qt5-dev=4.7-4 libboost-random-dev curl python-catkin-tools python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep ros-kinetic-tf2-geometry-msgs

# Use bash and create user
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN useradd -ms /bin/bash user_fsm
USER user_fsm
WORKDIR /home/user_fsm

RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/user_fsm/.bashrc
RUN echo "source /home/user_fsm/catkin_ws/devel/setup.bash" >> /home/user_fsm/.bashrc

RUN rosdep update
RUN mkdir -p /home/user_fsm/catkin_ws/src && \
    cd /home/user_fsm/catkin_ws/src/

COPY fsm/ /home/user_fsm/catkin_ws/src/fsm/

RUN cd /home/user_fsm/catkin_ws && \
    export CC=gcc && \
    export CXX=g++ && \
    alias g++='g++ -std=c++11' && \
    alias clang++='clang++ -std=c++11' && \
    source /opt/ros/kinetic/setup.bash && \
    catkin build fsm_lidom_ros && \
    source /opt/ros/kinetic/setup.bash && \
    source /home/user_fsm/catkin_ws/devel/setup.bash

# The next five lines + the entrypoint command will make sure that
# when the container is run the fsm node is roslaunched immediately
RUN echo "#!/bin/bash" > /home/user_fsm/fsm_launch.sh
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/user_fsm/fsm_launch.sh
RUN echo "source /home/user_fsm/catkin_ws/devel/setup.bash" >> /home/user_fsm/fsm_launch.sh
RUN echo "roslaunch fsm_lidom_ros avanti_fsm_lidom.launch" >> /home/user_fsm/fsm_launch.sh
RUN chmod +x /home/user_fsm/fsm_launch.sh

ENTRYPOINT bash /home/user_fsm/fsm_launch.sh
