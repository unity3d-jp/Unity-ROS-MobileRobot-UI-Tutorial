cmake_minimum_required(VERSION 2.8.3)
project(raspimouse_fake)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/clean_dev_file.sh
  scripts/gen_dev_file.sh
  scripts/virtual_led_sensors.py
  scripts/virtual_motors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch misc)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)