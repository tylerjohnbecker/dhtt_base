#include "dhtt_plugins/behaviors/cooking_interact_special_behavior.hpp"

namespace dhtt_plugins
{

double CookingInteractSpecialBehavior::get_perceived_efficiency()
{
	double activation_check = CookingBehavior::get_perceived_efficiency();

	if (activation_check != 0)
	{
		// High activation if we're right next to the object. No activation otherwise.
		double to_ret = this->agent_point_distance(this->destination_point) == 1.0 ? 1.0 : 0.0;
		RCLCPP_INFO(this->pub_node_ptr->get_logger(), "I report %f efficiency", to_ret);

		this->activation_potential = to_ret; // TODO is this necessary?
		return to_ret;
	}

	return 0;
}

void CookingInteractSpecialBehavior::do_work(dhtt::Node *container)
{
	(void)container; // Unused

	/* move_to */
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
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "move_to request did not succeed, returning early: %s",
					 res.future.get()->error_msg.c_str());
		this->done = false;
		return;
	}

	/* interact_special */
	req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
	req->action.action_type =
		dhtt_msgs::msg::CookingAction::INTERACT_PICK_UP_SPECIAL; // TODO change to explicit arm

	res = this->cooking_request_client->async_send_request(req);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "Sending interact_special request");
	this->executor->spin_until_future_complete(res);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "execute interact_special completed");

	suc = res.future.get()->success;
	if (not suc)
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "interact_special request did not succeed: %s",
					 res.future.get()->error_msg.c_str());
	}

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingInteractSpecialBehavior::get_retained_resources(dhtt::Node *container)
{
	std::vector<dhtt_msgs::msg::Resource> to_ret;

	// just keep access to the first gripper found (shouldn't matter for now)
	for (auto resource_iter : container->get_owned_resources())
	{
		if (resource_iter.type == dhtt_msgs::msg::Resource::GRIPPER)
		{
			to_ret.push_back(resource_iter);
			break;
		}
	}

	return to_ret;
}

std::vector<dhtt_msgs::msg::Resource>
CookingInteractSpecialBehavior::get_released_resources(dhtt::Node *container)
{
	std::vector<dhtt_msgs::msg::Resource> to_ret;
	bool first = true;

	for (auto resource_iter : container->get_owned_resources())
	{
		if (resource_iter.type != dhtt_msgs::msg::Resource::GRIPPER or not first)
		{
			to_ret.push_back(resource_iter);
		}
		else
		{ // skip the first gripper but not the rest
			first = false;
		}
	}

	return to_ret;
}

std::vector<dhtt_msgs::msg::Resource> CookingInteractSpecialBehavior::get_necessary_resources()
{
	std::vector<dhtt_msgs::msg::Resource> to_ret;

	dhtt_msgs::msg::Resource base;
	base.type = dhtt_msgs::msg::Resource::BASE;

	dhtt_msgs::msg::Resource gripper;
	gripper.type = dhtt_msgs::msg::Resource::GRIPPER;

	to_ret.push_back(base);
	to_ret.push_back(gripper);

	return to_ret;
}
} // namespace dhtt_plugins