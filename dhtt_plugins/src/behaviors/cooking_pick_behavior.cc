#include "dhtt_plugins/behaviors/cooking_pick_behavior.hpp"

namespace dhtt_plugins
{
double CookingPickBehavior::get_perceived_efficiency()
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

void CookingPickBehavior::do_work(dhtt::Node *container)
{
	(void)container; // Unused

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

	auto res = this->cooking_request_client->async_send_request(req);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "Sending move_to request");
	this->executor->spin_until_future_complete(res);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "move_to request completed");

	bool suc = res.get()->success;
	if (not suc)
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "move_to request did not succeed, returning early: %s",
					 res.get()->error_msg.c_str());
		this->done = false;
		return;
	}

	/* interact_primary */
	req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

	// see pr2.yaml
	req->action.action_type = CookingBehavior::which_arm(container) == "left_arm"
								  ? dhtt_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM1
								  : dhtt_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM2;

	res = this->cooking_request_client->async_send_request(req);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "Sending interact request");
	this->executor->spin_until_future_complete(res);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "execute interact completed");

	suc = res.get()->success;
	if (not suc)
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "interact_primary request did not succeed: %s",
					 res.get()->error_msg.c_str());
		return;
	}

	// if object is not marked for anyone
	if (not this->destination_mark.empty() and this->check_mark(this->destination_object) == '2')
	{
		RCLCPP_INFO(this->pub_node_ptr->get_logger(),
					("Marking object as " + this->destination_mark).c_str());
		suc = this->mark_object(this->destination_object.world_id, this->destination_mark);
		if (not suc)
		{
			RCLCPP_ERROR(this->pub_node_ptr->get_logger(), "Marking object failed: %s",
						 res.get()->error_msg.c_str());
		}
	}

	// Equivalent of get_released_resources() but for "resources" on the paramserver
	if (not this->destination_mark.empty() and this->check_mark(this->destination_object) == '1')
	{
		RCLCPP_INFO(this->pub_node_ptr->get_logger(),
					("Unmarking object that was marked as " + this->destination_mark + " under " +
					 this->destination_object.object_type)
						.c_str());
		suc = this->unmark_static_object_under_obj(this->destination_object);
		if (not suc)
		{
			RCLCPP_ERROR(this->pub_node_ptr->get_logger(), ("Error unmarking static object under " +
															this->destination_object.object_type)
															   .c_str());
		}
	}

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingPickBehavior::get_retained_resources(dhtt::Node *container)
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
CookingPickBehavior::get_released_resources(dhtt::Node *container)
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