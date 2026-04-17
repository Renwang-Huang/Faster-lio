#include <unistd.h>
#include <csignal>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "laser_mapping.h"
#include "utils.h"

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    RCLCPP_WARN(rclcpp::get_logger("faster_lio"), "Catch signal %d, exiting...", sig);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("faster_lio");
    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    
    if (!laser_mapping->InitROS(node)) {
        RCLCPP_ERROR(node->get_logger(), "Laser mapping init failed!");
        rclcpp::shutdown();
        return -1;
    }

    signal(SIGINT, SigHandle);
    rclcpp::Rate rate(5000);

    while (rclcpp::ok()) {
        if (faster_lio::options::FLAG_EXIT) break;
        rclcpp::spin_some(node);
        laser_mapping->Run();
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Finishing mapping...");
    faster_lio::Timer::PrintAll();

    rclcpp::shutdown();
    return 0;
}
