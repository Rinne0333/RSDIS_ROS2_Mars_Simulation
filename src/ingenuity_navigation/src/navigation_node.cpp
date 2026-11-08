/**
 * @file navigation_node.cpp
 * @brief Ponto de entrada: cria o nó e entrega-lhe o executor ROS 2.
 *
 * Toda a lógica de voo está distribuída pelos módulos de src/.
 */
#include "ingenuity_navigation/navigation.hpp"

#include <exception>
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;

    try
    {
        const auto node = std::make_shared<IngenuityNavigation>();
        rclcpp::spin(node);
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ingenuity_navigation"),
                     "Fatal error: %s", error.what());
        result = 1;
    }

    rclcpp::shutdown();
    return result;
}
