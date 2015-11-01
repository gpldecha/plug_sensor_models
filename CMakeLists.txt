cmake_minimum_required(VERSION 2.8.3)
project(peg_sensor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp optitrack_rviz netft_rdt_driver std_msgs world_wrapper wrap_object objects statistics_ml)

find_package(Armadillo)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")        ## Optimize
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")


#add_service_files(
#  FILES
#  String_cmd.srv
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp world_wrapper wrap_object statistics_ml objects
  DEPENDS system_lib
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES ${INCLUDE_DIR}/peg_sensor/peg_sensor_model/distance_features.h
                 ${INCLUDE_DIR}/peg_sensor/peg_sensor_model/distance_model.h
                 ${INCLUDE_DIR}/peg_sensor/peg_sensor_model/peg_distance_model.h
                 ${INCLUDE_DIR}/peg_sensor/peg_sensor_model/peg_sensor_model.h


                 ${INCLUDE_DIR}/peg_sensor/classifier/force_iid_model.h
                 ${INCLUDE_DIR}/peg_sensor/classifier/gmm_model.h
                 ${INCLUDE_DIR}/peg_sensor/classifier/peg_classifier_publisher.h
                 ${INCLUDE_DIR}/peg_sensor/classifier/base_peg_classifier.h
                 ${INCLUDE_DIR}/peg_sensor/classifier/peg_classifier.h

                 ${INCLUDE_DIR}/peg_sensor/peg_world_wrapper/peg_world_wrapper.h



                 ${INCLUDE_DIR}/peg_sensor_manager/sensor_manager.h
)


set(SRC_FILES    ${SRC_DIR}/peg_sensor_model/distance_features.cpp
                 ${SRC_DIR}/peg_sensor_model/distance_model.cpp
                 ${SRC_DIR}/peg_sensor_model/peg_distance_model.cpp
                 ${SRC_DIR}/peg_sensor_model/peg_sensor_model.cpp


                 ${SRC_DIR}/classifier/force_iid_model.cpp
                 ${SRC_DIR}/classifier/gmm_model.cpp
                 ${SRC_DIR}/classifier/peg_classifier_publisher.cpp
                 ${SRC_DIR}/classifier/peg_classifier.cpp

                 ${SRC_DIR}/peg_world_wrapper/peg_world_wrapper.cpp


                 ${SRC_DIR}/listener.cpp



                 ${SRC_DIR}/peg_sensor_manager/sensor_manager.cpp
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
  ${INCLUDE_STATS}
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES}  ${LIB_STATS} )

## Declare a cpp executable
add_executable(peg_sensor_classifier_node ${SRC_DIR}/peg_sensor_classifier_node.cpp)
target_link_libraries(peg_sensor_classifier_node ${catkin_LIBRARIES} ${PROJECT_NAME}  )

