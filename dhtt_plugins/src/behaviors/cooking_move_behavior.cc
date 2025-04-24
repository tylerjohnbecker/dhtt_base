#include "dhtt_plugins/behaviors/cooking_move_behavior.hpp"

namespace dhtt_plugins
{
double CookingMoveBehavior::get_perceived_efficiency()
{
	double activation_check = CookingBehavior::get_perceived_efficiency();

	if (activation_check != 0)
	{
		double to_ret = pow(1.0 + this->agent_point_distance(this->destination_point), -1);
		RCLCPP_INFO(this->pub_node_ptr->get_logger(), "I report %f efficiency", to_ret);

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

	auto req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
	req->action.action_type = dhtt_msgs::msg::CookingAction::MOVE_TO;

	std::string dest_point_str = std::to_string(this->destination_point.x) + ", " +
								 std::to_string(this->destination_point.y);

	req->action.params = dest_point_str;

	auto res = this->cooking_request_client->async_send_request(req);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "Sending move_to request");
	this->executor->spin_until_future_complete(res);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "move_to request completed");

	bool suc = res.future.get()->success;
	if (not suc)
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(), "move_to request did not succeed: %s",
					 res.future.get()->error_msg.c_str());
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