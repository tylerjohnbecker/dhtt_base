#ifndef TEST_BEHAVIOR_HPP
#define TEST_BEHAVIOR_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"

#include <vector>
#include <string>

namespace dhtt_plugins
{
	/**
	 * \brief A behavior which is just utilized for testing the functionality of the tree
	 */
	class TestBehavior : public dhtt::NodeType
	{
	public:

		void initialize(std::vector<std::string> params) override;

		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container ) override;

		void parse_params( std::vector<std::string> params ) override;

		double get_perceived_efficiency() override;

		bool is_done() override;

	protected:

		bool done;

		double activation_potential;
		std::vector<dhtt_msgs::msg::Resource> necessary_resources; 

	private:
	};
}

#endif //TEST_BEHAVIOR_HPP