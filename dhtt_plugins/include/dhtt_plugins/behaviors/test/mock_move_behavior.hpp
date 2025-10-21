#ifndef MOCK_MOVE_BEHAVIOR_HPP_
#define MOCK_MOVE_BEHAVIOR_HPP_

#include "dhtt_plugins/behaviors/action_type.hpp"
#include "std_msgs/msg/string.hpp"

namespace dhtt_plugins
{

	class MockMoveBehavior : public dhtt_plugins::ActionType
	{
	public:
		virtual void parse_params(std::vector<std::string> params) override;

		virtual void do_work( dhtt::Node* container ) override;
		
		virtual std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) override;
		virtual std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node* container ) override;
		virtual std::vector<dhtt_msgs::msg::Resource> get_necessary_resources() override;

		virtual double get_perceived_efficiency(dhtt::Node* container) override;
		
	private:
		bool work_done;

		double activation_potential;
		std::string destination;
	};

}

#endif // MOCK_MOVE_BEHAVIOR_HPP_