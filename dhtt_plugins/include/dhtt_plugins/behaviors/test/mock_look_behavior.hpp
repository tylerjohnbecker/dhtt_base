#ifndef MOCK_LOOK_BEHAVIOR_HPP_
#define MOCK_LOOK_BEHAVIOR_HPP_

#include "dhtt_plugins/behaviors/action_type.hpp"

namespace dhtt_plugins
{

	class MockLookBehavior : public dhtt_plugins::ActionType
	{
	public:
		virtual void parse_params(std::vector<std::string> params) override;

		virtual void do_work( dhtt::Node* container ) override;
		
		virtual std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) override;
		virtual std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node* container ) override;
		virtual std::vector<dhtt_msgs::msg::Resource> get_necessary_resources() override;

		virtual double get_perceived_efficiency(dhtt::Node* container) override;

	private:
		std::string object_target;

	};

}

#endif //MOCK_LOOK_BEHAVIOR_HPP_