FROM tiryoh/ros-desktop-vnc:melodic

ENV ROS_WORKSPACE=/home/ubuntu/catkin_ws

RUN apt-get update -q && \
    apt-get install -y dos2unix ros-melodic-joint-state-publisher-gui && \
    rm -rf /var/lib/apt/lists/*

COPY ./src/ros_tcp_endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint
COPY ./src/raspimouse_sim $ROS_WORKSPACE/src/raspimouse_sim
COPY ./src/raspimouse_description $ROS_WORKSPACE/src/raspimouse_description
COPY ./configure.sh /home/ubuntu/configure.sh

RUN cd ${ROS_WORKSPACE} && \
    /bin/bash -c "find $ROS_WORKSPACE -type f -print0 | xargs -0 dos2unix" && \
    dos2unix $HOME/configure.sh && \
    chown -R ubuntu:ubuntu /home/ubuntu

USER ubuntu
RUN cd $ROS_WORKSPACE && \
    . /opt/ros/melodic/setup.sh && \
    catkin init && \
    catkin build && \
    echo ". \$HOME/catkin_ws/devel/setup.bash" >> /home/ubuntu/.bashrc && \
    echo ". \$HOME/configure.sh" >> /home/ubuntu/.bashrc

USER root