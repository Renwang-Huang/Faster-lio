set(LIBRARY_NAME "${PROJECT_NAME}_lib")

add_library(${LIBRARY_NAME} SHARED
        laser_mapping.cc
        pointcloud_preprocess.cc
        options.cc
        utils.cc
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${LIBRARY_NAME} "${cpp_typesupport_target}")

ament_target_dependencies(${LIBRARY_NAME}
        rclcpp
        sensor_msgs
        nav_msgs
        geometry_msgs
        tf2
        tf2_ros
        tf2_eigen
        pcl_conversions
        livox_ros_driver2 
)

target_link_libraries(${LIBRARY_NAME}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog
        yaml-cpp
        Eigen3::Eigen
)

target_include_directories(${LIBRARY_NAME} PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
        ${PYTHON_INCLUDE_DIRS}
)

install(TARGETS ${LIBRARY_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)