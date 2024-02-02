#ifndef NODE_HPP
#define NODE_HPP

#include <memory>
#include <chrono>

#include "pluginlib/class_loader.hpp"

#include "dhtt_msgs/msg/node.hpp"
#include "dhtt_msgs/msg/resource.hpp"
#include "dhtt_msgs/msg/node_status.hpp"

#include "dhtt_msgs/srv/internal_service_registration.hpp"

#include "dhtt/tree/node_type.hpp"

#define TREE_PREFIX "/dhtt_tree/"
#define REGISTER_CHILD_POSTFIX "/register_child"

namespace dhtt
{
	class NodeType;

	class Node : public rclcpp::Node
	{
	public:
		Node(std::string name, std::string type, std::vector<std::string> params, std::string parent_name);
		~Node();

		bool loaded_successfully();
		std::string get_error_msg();

		std::vector<dhtt_msgs::msg::Resource> get_owned_resources();

		void register_with_parent();
		bool remove_child(std::string child_name);

	protected:

		// service callbacks
		void register_child_callback(std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Response> response);

		// services
		rclcpp::Service<dhtt_msgs::srv::InternalServiceRegistration>::SharedPtr register_server;

		// members
		pluginlib::ClassLoader<NodeType> node_type_loader;
		std::shared_ptr<NodeType> logic;

		dhtt_msgs::msg::NodeStatus status;

		std::vector<dhtt_msgs::msg::Resource> owned_resources;
		std::vector<std::string> child_names;

		std::string name;
		std::string parent_name;

		double activation_potential;

		// not sure if i need this
		int activation_level;		

		bool successful_load;
		std::string error_msg;

	private:
	};

}

#endif // NODE_HPP