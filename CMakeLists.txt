cmake_minimum_required(VERSION 2.8.3)
project(plug_sensor_models)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp optitrack_rviz netft_rdt_driver std_msgs world_wrapper wrap_object)

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
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

set(INCLUDE_STATS /home/guillaume/CppWorkSpace/Statistics/include)
set(LIB_STATS      /home/guillaume/CppWorkSpace/Statistics/lib/libstatistics.so)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES ${INCLUDE_DIR}/plug_sensor_models/force_iid_model.h
                 ${INCLUDE_DIR}/plug_sensor_models/gmm_model.h
                 ${INCLUDE_DIR}/plug_sensor_models/force_iid_model.h
                 ${INCLUDE_DIR}/plug_sensor_models/listener.h
                 ${INCLUDE_DIR}/plug_sensor_models/plug_distance_model.h
                 ${INCLUDE_DIR}/plug_sensor_models/distance_model.h
                 ${INCLUDE_DIR}/plug_sensor_models/distance_features.h
                 ${INCLUDE_DIR}/plug_sensor_manager/sensor_manager.h
)

set(SRC_FILES    ${SRC_DIR}/force_iid_model.cpp
                 ${SRC_DIR}/gmm_model.cpp
                 ${SRC_DIR}/force_iid_model.cpp
                 ${SRC_DIR}/distance_model.cpp
                 ${SRC_DIR}/listener.cpp
                 ${SRC_DIR}/distance_features.cpp
                 ${SRC_DIR}/plug_distance_model.cpp
                 ${SRC_DIR}/plug_sensor_manager/sensor_manager.cpp

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
  ${INCLUDE_STATS}
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    armadillo
    ${LIB_STATS}
)

## Declare a cpp executable
add_executable(plug_sensor_node ${SRC_DIR}/plug_sensor_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(plug_sensor_node plug_sensor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(plug_sensor_node    ${catkin_LIBRARIES}   ${PROJECT_NAME}  )

