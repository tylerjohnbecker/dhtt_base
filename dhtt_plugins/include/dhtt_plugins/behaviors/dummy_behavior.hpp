#ifndef DUMMY_BEHAVIOR_HPP
#define DUMMY_BEHAVIOR_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node_type.hpp"

#include <vector>
#include <string>

namespace dhtt_plugins
{
	class DummyBehavior : public dhtt::NodeType
	{
	public:

		void initialize(std::vector<std::string> params) override;

		void auction_callback( dhtt::Node& container ) override;
		void result_callback( dhtt::Node& container, bool success) override;

		void work() override;

		double get_perceived_efficiency() override;

		bool can_add_child() override;

		std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node& container ) override;
		std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node& container ) override;

	protected:

	private:
	};
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(dhtt_plugins::DummyBehavior, dhtt::NodeType)

#endif DUMMY_BEHAVIOR_HPP