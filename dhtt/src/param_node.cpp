
// ros2 standard libraries
#include "rclcpp/rclcpp.hpp"

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv); 

	std::shared_ptr<rclcpp::Node> param_ptr = std::make_shared<rclcpp::Node>("param_node", 
																	rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

	RCLCPP_INFO(param_ptr->get_logger(), "Listening for parameters...");

	param_ptr->declare_parameter("world.marked_objects_taints", std::vector<std::string>());
	param_ptr->declare_parameter("world.marked_objects_types", std::vector<std::string>());
	param_ptr->declare_parameter("world.marked_objects_ids", std::vector<long int>());

	rclcpp::spin(param_ptr);
	rclcpp::shutdown();

	return 0;
}