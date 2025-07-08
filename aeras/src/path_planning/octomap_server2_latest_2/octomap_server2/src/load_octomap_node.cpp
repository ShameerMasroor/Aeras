#include <octomap_server2/octomap_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <iostream>

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the OctomapServer node
    auto node = rclcpp::Node::make_shared("load_octomap_node");

    // Declare the parameter for the map file path
    node->declare_parameter<std::string>("map_file_path", "");

    // Get the map file path from the parameter
    std::string octomap_file;
    if (!node->get_parameter("map_file_path", octomap_file) || octomap_file.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'map_file_path' not provided or is empty.");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Loading Octomap file: %s", octomap_file.c_str());

    // Create the OctomapServer object
    auto octomap_server = std::make_shared<octomap_server::OctomapServer>(rclcpp::NodeOptions());

    // Load the OctoMap file
    if (!octomap_server->openFile(octomap_file)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load Octomap file: %s", octomap_file.c_str());
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Octomap file loaded successfully: %s", octomap_file.c_str());

    // Spin to process callbacks and publish map data
    rclcpp::spin(octomap_server);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
