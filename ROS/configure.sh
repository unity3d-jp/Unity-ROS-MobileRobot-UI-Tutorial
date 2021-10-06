#!/bin/bash -eu

ROS_IP=$(hostname -i)

echo "ROS_IP: $ROS_IP" > $(rospack find ros_tcp_endpoint)/config/params.yaml
sed -i -e "s/127\.0\.0\.1/$ROS_IP/g" $HOME/.bashrc
if [ -e $HOME/.ignition/fuel/config.yaml ]; then
    sed -i -e "s/ignitionfuel\.org/ignitionrobotics.org/g" $HOME/.ignition/fuel/config.yaml
fi

export ROS_IP="$ROS_IP"
export ROS_MASTER_URI="http://$ROS_IP:11311"