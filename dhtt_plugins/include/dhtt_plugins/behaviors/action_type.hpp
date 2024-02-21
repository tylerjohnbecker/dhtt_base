#ifndef ACTION_TYPE_HPP
#define ACTION_TYPE_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"

#include <vector>
#include <string>

namespace dhtt_plugins
{
	class ActionType : public dhtt::NodeType
	{
	public:

		void initialize(std::vector<std::string> params) override;

		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container ) override;

		virtual void do_work( dhtt::Node* container ) = 0;

		virtual std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) = 0;
		virtual std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node* container ) = 0;

		bool is_done() override;

	protected:

		bool done;

		double activation_potential;
		std::vector<dhtt_msgs::msg::Resource> necessary_resources; 

	private:
	};
}

#endif //ACTION_TYPE_HPP