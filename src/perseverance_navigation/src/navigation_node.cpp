#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "perseverance_navigation/navigation.hpp"


int main(int argc, char *argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create navigation node
    auto node = std::make_shared<PerseveranceNavigation>();

    // Execute ROS 2 callbacks
    rclcpp::spin(node);

    // Stop robot before shutdown
    if (rclcpp::ok())
    {
        node->stopRobot();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}