#ifndef OR_BEHAVIOR_HPP
#define OR_BEHAVIOR_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"

#include <vector>
#include <string>

namespace dhtt_plugins
{
	class OrBehavior : public dhtt::NodeType
	{
	public:

		void initialize(std::vector<std::string> params) override;

		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container ) override;

		void parse_params( std::vector<std::string> params ) override;

		double get_perceived_efficiency() override;


		bool is_done() override;

	protected:
		bool has_chosen_child;
		bool child_has_run;
		bool child_done;
		double activation_potential;

		std::string activated_child_name;
	private:
	};
}

#endif //OR_BEHAVIOR_HPP