#ifndef TEST_BEHAVIOR_HPP
#define TEST_BEHAVIOR_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node_type.hpp"

#include <vector>
#include <string>

namespace dhtt_plugins
{
	class TestBehavior : public dhtt::NodeType
	{
	public:

		void initialize(std::vector<std::string> params) override;

		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container, bool success) override;

		void parse_params( std::vector<std::string> params ) override;

		double get_perceived_efficiency() override;

		std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) override;
		std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node* container ) override;

	protected:

		double activation_potential;

	private:
	};
}

#include <pluginlib/class_list_macros.hpp>

#endif //DUMMY_BEHAVIOR_HPP