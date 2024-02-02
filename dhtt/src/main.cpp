/**
 * insert docs here
 */

// CPP standard libraries
#include <iostream>
#include <memory>

// ros2 standard libraries
#include "rclcpp/rclcpp.hpp"

// dHTT includes
#include "dhtt/server/main_server.hpp"

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

	auto my_server = std::make_shared<dhtt::MainServer>("dHTT_server", spinner);

	RCLCPP_INFO(my_server->get_logger(), "Server started...");

	spinner->add_node(my_server);

	spinner->spin();

	return 0;
}