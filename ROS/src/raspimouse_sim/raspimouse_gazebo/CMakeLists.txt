cmake_minimum_required(VERSION 2.8.3)
project(raspimouse_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/download_gazebo_models.sh
  scripts/distance_sensor_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch worlds)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)