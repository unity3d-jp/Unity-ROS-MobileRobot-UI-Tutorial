#!/usr/bin/env bash
set -eu

WORKINGDIR="$(rospack find raspimouse_gazebo)"

rosrun xacro xacro --inorder $WORKINGDIR/materials/sample_maze.world.xacro > $WORKINGDIR/worlds/sample_maze.world