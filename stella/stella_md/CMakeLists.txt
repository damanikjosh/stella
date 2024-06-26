cmake_minimum_required(VERSION 3.0.2)
project(stella_md)

set(CMAKE_C_COMPILER "g++")

find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_msgs 
)

#catkin_package(
 # CATKIN_DEPENDS
 # genmsg
 # roscpp
 # rospy
 # std_msgs
 #can_msgs
#)



#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES std_msgs geometry_msgs nav_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include/mw"
  "${PROJECT_SOURCE_DIR}/include/serial"
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mw_test.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/main.cpp serial/list_ports_linux.cc serial/serial.cc serial/unix.cc mw/stella.cpp)




## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
 )

# Detect the architecture
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Initialize the variable for the library
set(MOTOR_DRIVER_LIB "")

# Conditionally link the correct library
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(MOTOR_DRIVER_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/MW_MotorDriver_aarch64.a")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64")
    set(MOTOR_DRIVER_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/MW_MotorDriver_arm64.a")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l")
    set(MOTOR_DRIVER_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/MW_MotorDriver_armv7l.a")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "amd64")
    set(MOTOR_DRIVER_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/MW_MotorDriver_amd64.a")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

target_link_libraries(${PROJECT_NAME}_node
  ${MOTOR_DRIVER_LIB}
  ${catkin_LIBRARIES}
)



install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY rviz DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mw_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
