cmake_minimum_required(VERSION 2.8.3)
project(downward)

catkin_package()

add_custom_command(
  OUTPUT installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile
)
add_custom_command(
  OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  COMMAND cmake -E copy_directory downward/scripts/ ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  DEPENDS installed
)

add_custom_target(downward_install ALL DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

