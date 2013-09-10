cmake_minimum_required(VERSION 2.8.3)

project(infinisoleil)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater dynamic_reconfigure nodelet roscpp sensor_msgs
  roslaunch rostest)

# fx8_driver.so depends Boost threads.
find_package(Boost REQUIRED COMPONENTS thread)

# Include path of the Infinisoleil library.
find_path(
  LIBFX8_INCLUDE_DIR fx8.h
  PATHS
    /usr/local/include/infinisoleil/
)

# Library path of the Infinisoleil library.
find_library(
  LIBFX8_LIBRARY libfx8.so
  PATHS
    /usr/local/lib/
)

if(NOT LIBFX8_INCLUDE_DIR OR NOT LIBFX8_LIBRARY)
  message(SEND_ERROR "Infinisoleil library is not found.")
endif(NOT LIBFX8_INCLUDE_DIR OR NOT LIBFX8_LIBRARY)

generate_dynamic_reconfigure_options(
  cfg/FX8.cfg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LIBFX8_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS diagnostic_updater dynamic_reconfigure nodelet
  roscpp sensor_msgs
)

add_library(fx8_driver src/nodelet/fx8_driver_nodelet.cpp)

add_executable(fx8_node src/node/fx8_node.cpp)

add_dependencies(fx8_driver ${PROJECT_NAME}_gencfg)

target_link_libraries(fx8_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBFX8_LIBRARY}
)

target_link_libraries(fx8_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# installation

install(TARGETS fx8_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS fx8_driver
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES infinisoleil_nodelets.xml package.xml COPYING
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Tests have two purposes. One is to check fps corresponded measure
# mode of an Infinisoleil sensor. Frame cycles according to measure
# modes differ at each sensor. Another is to check that publishers work
# correctly.

install(DIRECTORY tests/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)
