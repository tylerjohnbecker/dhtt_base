#ifndef NODE_TYPE_HPP
#define NODE_TYPE_HPP

// INCLUDES
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node.hpp"

namespace dhtt
{

	enum goal_t
	{
		PERSISTENT,
		TRANSIENT
	};

	class Node;

	class NodeType
	{
	public:

		virtual void initialize(std::vector<std::string> params) = 0;

		virtual void auction_callback( Node& container ) = 0;
		virtual void result_callback( Node& container, bool success) = 0;

		virtual void work() = 0;

		virtual double get_perceived_efficiency() = 0;

		virtual std::vector<dhtt_msgs::msg::Resource> get_retained_resources( Node& container ) = 0;
		virtual std::vector<dhtt_msgs::msg::Resource> get_released_resources( Node& container ) = 0;

		virtual bool can_add_child() {return false;};

		goal_t goal_type = PERSISTENT;

		std::vector<dhtt_msgs::msg::Resource> necessary_resources;

	protected:
	};

}

#endif //NODE_TYPE_HPP