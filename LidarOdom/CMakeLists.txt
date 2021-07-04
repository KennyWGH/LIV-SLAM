cmake_minimum_required(VERSION 3.0.2)
project(lidar_odom)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## 添加版本号
set(LIDAR_ODOM_MAJOR_VERSION 1)
set(LIDAR_ODOM_MINOR_VERSION 0)
set(LIDAR_ODOM_PATCH_VERSION 0)
set(LIDAR_ODOM_VERSION ${LIDAR_ODOM_MAJOR_VERSION}.${LIDAR_ODOM_MINOR_VERSION}.${LIDAR_ODOM_PATCH_VERSION}) # 库版本号
set(LIDAR_ODOM_SOVERSION ${LIDAR_ODOM_MAJOR_VERSION}.${LIDAR_ODOM_MINOR_VERSION}.${LIDAR_ODOM_PATCH_VERSION}) # API版本号

## Find catkin macros and libraries and other dependices through CMmake's conventions
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  pcl_conversions
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS system filesystem thread iostreams chrono)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3.4.8 REQUIRED)
message(STATUS "wgh-------------------------------------print include boost: " ${Boost_INCLUDE_DIRS})
# message(STATUS "wgh-------------------------------------print libs of boost: " ${Boost_LIBRARIES})
message(STATUS "wgh-------------------------------------print include eigen3: " ${EIGEN3_INCLUDE_DIR})
# message(STATUS "wgh-------------------------------------print libs of eigen3: " ${EIGEN3_LIBRARIES})
message(STATUS "wgh-------------------------------------print include opencv: " ${OpenCV_INCLUDE_DIRS})
# message(STATUS "wgh-------------------------------------print libs of opencv: " ${OpenCV_LIBS})
message(STATUS "wgh-------------------------------------print include pcl: " ${PCL_INCLUDE_DIRS})
# message(STATUS "wgh-------------------------------------print libs of pcl: " ${PCL_LIBRARIES})
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV >= 3.4 not found.")
endif()



###################################
## catkin specific configuration ##
###################################
# wgh-- 当作者发布了自己的catkin包，供别人调用时，作者必须设置好
# wgh-- 自己的头文件，库文件，以及自己依赖的其他catkin包或系统依赖项。
# wgh-- catkin_package()正是服务于这个目的的。
# wgh-- 第1、2行制定了所发布包的头文件和库文件；
# wgh-- 第3行指定了所发布包依赖的其它catkin包；
# wgh-- 第4行指定了所发布包依赖的其它操作系统中的依赖项。
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_odom
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library, executable, etc.
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/LidarOdom.cpp
# )

## Fetch all soruce files. //定义所有需要编译为库文件的源文件
file(GLOB_RECURSE ALL_LIBRARY_HDRS "src/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "src/*.cpp")
file(GLOB_RECURSE ALL_EXECUTABLES "src/*_node.cpp")
file(GLOB_RECURSE ALL_TEST_FILES "src/*_test.cpp")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXECUTABLES} ${ALL_TEST_FILES})
message(STATUS "wgh--------  ALL_EXECUTABLES: " ${ALL_EXECUTABLES})
message(STATUS "wgh-------- ALL_LIBRARY_SRCS: " ${ALL_LIBRARY_SRCS})

# add_library(${PROJECT_NAME} STATIC
#     ${ALL_LIBRARY_HDRS}
#     ${ALL_LIBRARY_SRCS}
# )

# target_link_libraries(${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ${OpenCV_LIBS}
# )

add_executable(${PROJECT_NAME}_node
    # src/LidarOdom_node.cpp
    # src/liv_Options.cpp
    # src/LidarOdomWrapper.cpp
    # src/LidarOdom.cpp
    # src/liv_time.cpp
    # src/liv_utils.cpp
    # src/DynamicObjFilter.cpp
    ${ALL_EXECUTABLES}
    ${ALL_LIBRARY_SRCS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)




