#ifndef THEN_BEHAVIOR_HPP
#define THEN_BEHAVIOR_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"

#include <vector>
#include <algorithm>
#include <string>

namespace dhtt_plugins
{
	class ThenBehavior : public dhtt::NodeType
	{
	public:
		void initialize(std::vector<std::string> params) override;

		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container ) override;

		void parse_params( std::vector<std::string> params ) override;

		double get_perceived_efficiency() override;

		bool is_done() override;

	protected:
		double activation_potential;

		int child_queue_index;
		int child_queue_size;
		bool started_activation;
	private:
	};
}

#endif //THEN_BEHAVIOR_HPP