#include "dhtt_cooking_plugins/behaviors/cooking_move_behavior.hpp"

namespace dhtt_cooking_plugins
{
double CookingMoveBehavior::get_perceived_efficiency(dhtt::Node* container)
{
	double activation_check = CookingBehavior::get_perceived_efficiency(container);

	if (activation_check != 0)
	{
		double to_ret = pow(1.0 + this->agent_point_distance(this->destination_point), -1);

		// if (this->should_unmark)
		// {
		// 	to_ret *= 2;

		// 	if (to_ret > 2)
		// 		to_ret = 2;
		// }

		this->activation_potential = to_ret; // TODO is this necessary?
		return to_ret;
	}

	return 0;
}

void CookingMoveBehavior::do_work(dhtt::Node *container)
{
	(void)container; // Unused

	if (not CookingBehavior::can_work())
	{
		return;
	}

	auto req = std::make_shared<dhtt_cooking_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_cooking_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_cooking_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
	req->action.action_type = dhtt_cooking_msgs::msg::CookingAction::MOVE_TO;

	std::string dest_point_str = std::to_string(this->destination_point.x) + ", " +
								 std::to_string(this->destination_point.y);

	req->action.params = dest_point_str;
	auto res = this->send_request_and_update(req);

	bool suc = res.get()->success;
	if (not suc)
	{
		DHTT_LOG_ERROR(this->com_agg, "move_to request did not succeed: " <<
					 res.get()->error_msg);
		return;
	}

	// if object is not marked for anyone
	if (not this->destination_mark.empty() and this->check_mark(this->destination_object) == '2')
	{
		DHTT_LOG_INFO(this->com_agg, 
					"Marking object as " << this->destination_mark);
		suc = this->mark_object(this->destination_object.world_id, this->destination_mark);
		if (not suc)
		{
			DHTT_LOG_ERROR(this->com_agg, "Marking object failed: " <<
						 res.get()->error_msg);
		}
	}

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingMoveBehavior::get_retained_resources(dhtt::Node *container)
{
	return container->get_owned_resources();
}

std::vector<dhtt_msgs::msg::Resource>
CookingMoveBehavior::get_released_resources(dhtt::Node *container)
{
	(void)container; // unused
	return {};
}
std::vector<dhtt_msgs::msg::Resource> CookingMoveBehavior::get_necessary_resources()
{
	std::vector<dhtt_msgs::msg::Resource> to_ret;

	dhtt_msgs::msg::Resource base;
	base.type = dhtt_msgs::msg::Resource::BASE;

	// TODO sometimes seems to help
	dhtt_msgs::msg::Resource gripper;
	gripper.type = dhtt_msgs::msg::Resource::GRIPPER;

	to_ret.push_back(base);
	to_ret.push_back(gripper);

	return to_ret;
}

} // namespace dhtt_plugins