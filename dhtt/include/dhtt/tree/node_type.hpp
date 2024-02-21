#ifndef NODE_TYPE_HPP
#define NODE_TYPE_HPP

// INCLUDES
#include <vector>
#include <string>

// #include "dhtt/tree/node.hpp"

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

		virtual std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( Node* container ) = 0;
		virtual std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( Node* container ) = 0;

		virtual void parse_params(std::vector<std::string> params) = 0;

		virtual double get_perceived_efficiency() = 0;
		
		virtual bool is_done() {return true;};
		virtual bool can_add_child() {return this->children_allowed;};

		goal_t goal_type = PERSISTENT;

		std::vector<dhtt_msgs::msg::Resource> necessary_resources;

		std::vector<std::string> params;
		std::vector<std::string> preconditions;

	protected:

		bool children_allowed;
	};

}

#endif //NODE_TYPE_HPP