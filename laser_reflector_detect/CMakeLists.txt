cmake_minimum_required(VERSION 2.8.3)
project(laser_reflector_detect)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")#


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
    geometry_msgs
    message_generation
    sensor_msgs
    nav_msgs
    cartographer_ros_msgs
    visualization_msgs
    
    tf
    tf2
    tf2_eigen
    tf2_ros
    )
add_service_files(
    FILES
    igk_reflector_srv.srv
)
add_message_files(
    FILES
    reflector.msg
    reflectordata.msg
    )
    
# 生成消息时依赖于 std_msgs
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES realsense_test
     CATKIN_DEPENDS roscpp std_msgs message_runtime message_generation tf tf2 tf2_ros tf2_eigen nav_msgs sensor_msgs geometry_msgs visualization_msgs
    #  DEPENDS system_lib
    )


    
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

##############################################################################
# File Content
##############################################################################
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/ros_qt5/*.h )
#using below command to involve the sub-folder
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/realsense_test/*.h )

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################
file(GLOB QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
list(REMOVE_ITEM QT_SOURCES src/reflector_slam_node.cpp src/reflector_localization_node.cpp)

# Reflector SLAM and Localization sources
set(REFLECTOR_SLAM_SOURCES
    src/reflector_slam_node.cpp
)

set(REFLECTOR_LOCALIZATION_SOURCES
    src/reflector_localization_node.cpp
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    #${realsense2_INCLUDE_DIRS}
    #${PCL_INCLUDE_DIRS}
    #${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}   # 添加这一行
    ${tf2_INCLUDE_DIRS}        # 添加这一行
    ${tf_INCLUDE_DIRS}         # 添加这一行
    ${EIGEN3_INCLUDE_DIR}
    )


    
add_executable(${PROJECT_NAME}_node ${QT_SOURCES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
set(DEPENDENCIES ${realsense2_LIBRARY}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${catkin_LIBRARIES}
    Qt5::Widgets
    ${CERES_LIBRARIES}
    )

target_link_libraries(${PROJECT_NAME}_node ${DEPENDENCIES})

##############################################################################
# Reflector SLAM Node (without cartographer)
##############################################################################
add_executable(reflector_slam_node ${REFLECTOR_SLAM_SOURCES})
add_dependencies(reflector_slam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reflector_slam_node 
    ${catkin_LIBRARIES} 
    ${CERES_LIBRARIES}
    ${tf2_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf_LIBRARIES}
)

##############################################################################
# Reflector Localization Node (without cartographer)
##############################################################################
add_executable(reflector_localization_node ${REFLECTOR_LOCALIZATION_SOURCES})
add_dependencies(reflector_localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(reflector_localization_node PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(reflector_localization_node 
    ${catkin_LIBRARIES}
    ${tf2_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf_LIBRARIES}
)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY param  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}_node reflector_slam_node reflector_localization_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


