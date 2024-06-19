#ifndef MOVE_BEHAVIOR_HPP
#define MOVE_BEHAVIOR_HPP

#include "dhtt_plugins/behaviors/action_type.hpp"
#include "std_msgs/msg/string.hpp"

namespace dhtt_plugins
{

	class MoveBehavior : public ActionType
	{
	public:
		void parse_params(std::vector<std::string> params) override;

		void do_work( dhtt::Node* container ) override;

		double get_perceived_efficiency() override;

		std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) override;
		std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node* container ) override; 

		void done_callback( const std::shared_ptr<std_msgs::msg::String> data );

	protected:
		bool work_done;

		double activation_potential;
		std::string destination;
	};

}

#endif // MOVE_BEHAVIOR_HPP