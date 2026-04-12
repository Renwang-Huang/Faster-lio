cmake_minimum_required(VERSION 3.8) 
project(faster_lio)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)

set(DEFAULT_BUILD_TYPE "Release")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

find_package(gflags REQUIRED)
find_package(glog REQUIRED)

include(cmake/packages.cmake)

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)
if (WITH_IVOX_NODE_TYPE_PHC)
    add_definitions(-DIVOX_NODE_TYPE_PHC)
endif()

set(LIBRARY_TARGET_NAME ${PROJECT_NAME}_lib)

add_library(${LIBRARY_TARGET_NAME} SHARED
        src/laser_mapping.cc
        src/pointcloud_preprocess.cc
        src/options.cc
        src/utils.cc
        )

ament_target_dependencies(${LIBRARY_TARGET_NAME}
        rclcpp
        geometry_msgs
        nav_msgs
        sensor_msgs
        std_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_eigen
        pcl_conversions
        livox_ros_driver2
        )

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${LIBRARY_TARGET_NAME}
        ${PCL_LIBRARIES}
        gflags
        glog
        tbb
        yaml-cpp
        "${cpp_typesupport_target}"
        )

add_executable(run_mapping_online app/run_mapping_online.cc)
ament_target_dependencies(run_mapping_online
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    livox_ros_driver2
)
target_link_libraries(run_mapping_online
    ${LIBRARY_TARGET_NAME}
    gflags
    glog
    ${PCL_LIBRARIES}
)

install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(DIRECTORY cmake config launch rviz DESTINATION share/${PROJECT_NAME})

install(TARGETS ${LIBRARY_TARGET_NAME}
    EXPORT export_${LIBRARY_TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS 
    run_mapping_online 
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_TARGET_NAME})
ament_export_dependencies(
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
    tf2 tf2_ros tf2_eigen tf2_geometry_msgs
    pcl_conversions livox_ros_driver2 rosbag2_cpp
) 

ament_package()