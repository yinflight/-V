cmake_minimum_required(VERSION 2.8.3)
project(CAN_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
FILES
  Motor_Feedback.msg
  Motor_Control.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
#set(Qt5_DIR /home/Qt5.13.0/5.13.0/gcc_64/lib/cmake/Qt5)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find_package(Qt5Widgets REQUIRED)
#qt5_wrap_cpp(MOC src/qt_ros_test.h)
#qt5_wrap_ui(UIC src/qt_ros_test.ui)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES client_plugin
   CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  INCLUDE_DIRS include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ executable
#add_executable(${PROJECT_NAME}_node src/gui_qt.h src/gui_qt.ui)
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} Qt5::Widgets)
#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})

#add_executable(${PROJECT_NAME}_pub )
#target_link_libraries(${PROJECT_NAME}_pub ${catkin_LIBRARIES})

#add_executable(${PROJECT_NAME}_sub src/Motor_CAN.cpp)
#target_link_libraries(${PROJECT_NAME}_sub ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})
#add_executable(Motor_CAN src/Motor_CAN.cpp)
#add_dependencies(Motor_CAN control_generate_message_cpp)
#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})
#target_link_libraries(Motor_CAN ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})

#target_link_libraries(${PROJECT_NAME}_sub ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})
#add_executable(${PROJECT_NAME}_ser )
#target_link_libraries(${PROJECT_NAME}_ser ${catkin_LIBRARIES})




#Motor_CAN
add_executable(Motor_CAN src/Motor_CAN.cpp)
add_dependencies(Motor_CAN control_generate_messages_cpp)
target_link_libraries(Motor_CAN ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})

#Motor_Control
add_executable(Motor_Control src/Motor_Control.cpp)
add_dependencies(Motor_Control control_generate_messages_cpp)
target_link_libraries(Motor_Control ${catkin_LIBRARIES})



