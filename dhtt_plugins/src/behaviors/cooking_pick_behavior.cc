#include "dhtt_plugins/behaviors/cooking_pick_behavior.hpp"

namespace dhtt_plugins
{
double CookingPickBehavior::get_perceived_efficiency(dhtt::Node* container)
{
	double activation_check = CookingBehavior::get_perceived_efficiency(container);

	if (activation_check != 0)
	{
		// High activation if we're right next to the object. No activation otherwise.
		double to_ret = this->agent_point_distance(this->destination_point) == 1.0 ? 1.0 : 0.0;

		this->activation_potential = to_ret; // TODO is this necessary?
		return to_ret;
	}

	return 0;
}

void CookingPickBehavior::do_work(dhtt::Node *container)
{
	(void)container; // Unused

	std::vector<dhtt_msgs::msg::CookingObject> prev_held = this->last_obs->agents[0].holding;

	if (not CookingBehavior::can_work())
	{
		return;
	}

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

	// Equivalent of get_released_resources() but for "resources" on the paramserver
	RCLCPP_INFO(container->get_logger(),
				("Unmarking object that was marked as " + this->destination_mark + " under " +
				 this->destination_object.object_type)
					.c_str());
	if (not this->unmark_static_object_under_obj(this->destination_object))
	{
		RCLCPP_WARN(container->get_logger(),
					("Error unmarking static object under " + this->destination_object.object_type)
						.c_str());
	}

	/* interact_primary */
	req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

	// see pr2.yaml
	req->action.action_type = CookingBehavior::which_arm(container) == "left_arm"
								  ? dhtt_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM1
								  : dhtt_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM2;

	res = this->send_request_and_update(req);
	// res = this->cooking_request_client->async_send_request(req);
	// RCLCPP_INFO(container->get_logger(), "Sending interact request");
	// this->com_agg->spin_until_future_complete<std::shared_ptr<dhtt_msgs::srv::CookingRequest::Response>>(res);
	// RCLCPP_INFO(container->get_logger(), "execute interact completed");

	suc = res.get()->success;
	if (not suc)
	{
		RCLCPP_ERROR(container->get_logger(),
					 "interact_primary request did not succeed: %s", res.get()->error_msg.c_str());
		return;
	}

	// Mark the object that was picked. If the static object had multiple objects on top (chopping
	// bread), it is ambiguous which one will be picked.
	dhtt_msgs::msg::CookingObject picked_obj;
	for (const auto &held : this->last_obs->agents[0].holding)
	{
		if (std::find(prev_held.cbegin(), prev_held.cend(), held) == prev_held.cend())
		{
			picked_obj = held;
			break; // assume we can only pick one object
		}
	}

	// if picked object is not marked for anyone
	if (not this->destination_mark.empty() and this->check_mark(picked_obj) == '2')
	{
		RCLCPP_INFO(container->get_logger(),
					("Marking object as " + this->destination_mark).c_str());
		suc = this->mark_object(picked_obj.world_id, this->destination_mark);
		if (not suc)
		{
			RCLCPP_ERROR(container->get_logger(), "Marking object failed: %s",
						 res.get()->error_msg.c_str());
		}
	}

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingPickBehavior::get_retained_resources(dhtt::Node *container)
{
	if (this->should_unmark)
	{
		return container->get_owned_resources();
	}

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
CookingPickBehavior::get_released_resources(dhtt::Node *container)
{
	if (this->should_unmark)
	{
		return std::vector<dhtt_msgs::msg::Resource>();
	}

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

std::vector<dhtt_msgs::msg::Resource> CookingPickBehavior::get_necessary_resources()
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