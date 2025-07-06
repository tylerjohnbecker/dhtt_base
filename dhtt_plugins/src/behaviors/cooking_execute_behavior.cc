#include "dhtt_plugins/behaviors/cooking_execute_behavior.hpp"

namespace dhtt_plugins
{
double CookingExecuteBehavior::get_perceived_efficiency(dhtt::Node* container)
{
	this->activation_potential = CookingBehavior::get_perceived_efficiency(container);
	return this->activation_potential;
}

void CookingExecuteBehavior::do_work(dhtt::Node *container)
{
	(void)container; // Unused

	if (not CookingBehavior::can_work())
	{
		return;
	}

	const auto prev_dest_obj = this->destination_object;

	/* move_to */
	auto req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
	req->action.action_type = dhtt_msgs::msg::CookingAction::MOVE_TO;

	std::string dest_point_str = std::to_string(this->destination_point.x) + ", " +
								 std::to_string(this->destination_point.y);

	req->action.params = dest_point_str;

	auto res = this->send_request_and_update(req);
	// auto res = this->cooking_request_client->async_send_request(req);
	// RCLCPP_INFO(container->get_logger(), "Sending move_to request");
	// this->com_agg->spin_until_future_complete<std::shared_ptr<dhtt_msgs::srv::CookingRequest::Response>>(res);
	// RCLCPP_INFO(container->get_logger(), "move_to request completed");

	bool suc = res.get()->success;
	if (not suc)
	{
		RCLCPP_ERROR(container->get_logger(),
					 "move_to request did not succeed, returning early: %s",
					 res.get()->error_msg.c_str());
		this->done = false;
		return;
	}

	/* execute_action */
	req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

	// see pr2.yaml
	req->action.action_type = CookingBehavior::which_arm(container) == "left_arm"
								  ? dhtt_msgs::msg::CookingAction::EXECUTE_ACTION_ARM1
								  : dhtt_msgs::msg::CookingAction::EXECUTE_ACTION_ARM2;

	res = this->send_request_and_update(req);

	// res = this->cooking_request_client->async_send_request(req);
	// RCLCPP_INFO(container->get_logger(), "Sending execute request");
	// this->com_agg->spin_until_future_complete<std::shared_ptr<dhtt_msgs::srv::CookingRequest::Response>>(res);
	// RCLCPP_INFO(container->get_logger(), "execute request completed");

	// unmark everything at the object
	if (this->should_unmark)
	{
		// unmark everything at destination location (before it gets changed by observation
		// callback)
		// this->com_agg->spin_some();

		for (const auto &world_obj : this->last_obs->objects)
		{
			if (world_obj.location == prev_dest_obj.location)
			{
				if (not CookingBehavior::unmark_object(world_obj.world_id))
				{
					// suc = false; not actually an error when unmarking an unmarked object
					RCLCPP_ERROR(container->get_logger(),
								 ("Error unmarking object " + prev_dest_obj.object_type).c_str());
				}
			}
		}
	}

	suc = res.get()->success;
	if (not suc)
	{
		RCLCPP_ERROR(container->get_logger(), "execute_action request did not succeed: %s",
					 res.get()->error_msg.c_str());
	}

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingExecuteBehavior::get_retained_resources(dhtt::Node *container)
{
	if (this->should_unmark)
	{
		// keep the move base
		dhtt_msgs::msg::Resource to_keep;
		for (auto iter : container->get_owned_resources())
			if (iter.type == dhtt_msgs::msg::Resource::BASE)
				to_keep = iter;

		return {to_keep};
	}
	return container->get_owned_resources();
}

std::vector<dhtt_msgs::msg::Resource>
CookingExecuteBehavior::get_released_resources(dhtt::Node *container)
{
	// release the gripper
	if (this->should_unmark)
	{
		dhtt_msgs::msg::Resource to_release;
		for (auto iter : container->get_owned_resources())
			if (iter.type == dhtt_msgs::msg::Resource::GRIPPER)
				to_release = iter;

		return {to_release};
	}

	return {};
}

std::vector<dhtt_msgs::msg::Resource> CookingExecuteBehavior::get_necessary_resources()
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