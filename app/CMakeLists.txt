add_executable(run_mapping_online run_mapping_online.cc)

target_link_libraries(run_mapping_online
        ${PROJECT_NAME}_lib
        gflags
)

ament_target_dependencies(run_mapping_online 
        rclcpp 
        sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(run_mapping_online "${cpp_typesupport_target}")

install(TARGETS
        run_mapping_online
        DESTINATION lib/${PROJECT_NAME}
)
