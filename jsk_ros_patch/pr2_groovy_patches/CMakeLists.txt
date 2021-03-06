if("$ENV{ROS_Distributions}" STREQUAL "electric" OR "$ENV{ROS_DISTRO}" STREQUAL "fuerte" OR "$ENV{ROS_DISTRO}" STREQUAL "hydro")
  message("-- pr2_groovy_patches package is for groovy")
  return()
endif()

cmake_minimum_required(VERSION 2.8.3)
project(pr2_groovy_patches)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

macro(patch package)
  set(PATCH_BATCH_FILE ${PROJECT_SOURCE_DIR}/${package}_patch.bat )
  file(WRITE ${PATCH_BATCH_FILE} "set -x\npatch -p0 -N -i ${PROJECT_SOURCE_DIR}/patches/${package}.patch")
  execute_process(COMMAND rospack find ${package} OUTPUT_VARIABLE ${package}_PACKAGE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process( COMMAND cmake -E chdir ${${package}_PACKAGE_PATH} sh ${PATCH_BATCH_FILE} RESULT_VARIABLE RESULT_PATCH )
  message( STATUS "Patch result: ${RESULT_PATCH}, SOURCE DIR: ${${package}_PACKAGE_PATH}" )
  file(REMOVE ${PATCH_BATCH_FILE})
endmacro(patch package)

patch(face_detector_mono) 
patch(SnapMapICP)
patch(safe_teleop_base) # https://sourceforge.net/p/bosch-ros-pkg/tickets/9/

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES patches_for_pr2groovy
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(patches_for_pr2groovy
#   src/${PROJECT_NAME}/patches_for_pr2groovy.cpp
# )

## Declare a cpp executable
# add_executable(patches_for_pr2groovy_node src/patches_for_pr2groovy_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(patches_for_pr2groovy_node patches_for_pr2groovy_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(patches_for_pr2groovy_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS patches_for_pr2groovy patches_for_pr2groovy_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_patches_for_pr2groovy.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
