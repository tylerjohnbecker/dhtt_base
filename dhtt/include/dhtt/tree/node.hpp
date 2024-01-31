#ifndef NODE_HPP
#define NODE_HPP

#include <memory>

#include "pluginlib/class_loader.hpp"

#include "dhtt_msgs/msg/node.hpp"
#include "dhtt_msgs/msg/resource.hpp"
#include "dhtt_msgs/msg/node_status.hpp"

#include "dhtt/tree/node_type.hpp"

namespace dhtt
{
	class NodeType;

	class Node
	{
	public:
		Node(std::string type, std::vector<std::string> params, std::string parent_name);
		~Node();

		bool loaded_successfully();
		std::string get_error_msg();

		std::vector<dhtt_msgs::msg::Resource> get_owned_resources();

	protected:

		std::shared_ptr<NodeType> logic;

		dhtt_msgs::msg::NodeStatus status;

		std::vector<dhtt_msgs::msg::Resource> owned_resources;
		std::vector<std::string> child_names;

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