#include "dhtt_plugins/behaviors/cooking_place_behavior.hpp"

namespace dhtt_plugins
{
double CookingPlaceBehavior::get_perceived_efficiency(dhtt::Node* container)
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

void CookingPlaceBehavior::do_work(dhtt::Node *container)
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

	/* if placing on a deliversquare, send an extra nop so the deliversquare clears and unmark it if
	 * needed*/
	if (this->destination_object.object_type.find("Deliversquare") != std::string::npos)
	{
		req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
		req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
		req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

		// see pr2.yaml
		req->action.action_type = dhtt_msgs::msg::CookingAction::NO_OP;

		// res = this->cooking_request_client->async_send_request(req);
		// RCLCPP_INFO(container->get_logger(), "Sending NOP request");
		// this->com_agg->spin_until_future_complete<std::shared_ptr<dhtt_msgs::srv::CookingRequest::Response>>(res);
		// RCLCPP_INFO(container->get_logger(), "NOP interact completed");

		res = this->send_request_and_update(req);

		suc = res.get()->success;
		if (not suc)
		{
			RCLCPP_ERROR(container->get_logger(), "NOP request did not succeed: %s",
						 res.get()->error_msg.c_str());
			return;
		}

		// Equivalent of get_released_resources() but for "resources" on the paramserver
		if (not this->destination_mark.empty() and
			this->check_mark(this->destination_object) == '1')
		{
			RCLCPP_INFO(container->get_logger(),
						("Unmarking object that was marked as " + this->destination_mark +
						 " under " + this->destination_object.object_type)
							.c_str());
			suc = this->unmark_static_object_under_obj(this->destination_object);
			if (not suc)
			{
				RCLCPP_ERROR(
					container->get_logger(),
					("Error unmarking static object under " + this->destination_object.object_type)
						.c_str());
			}
		}
	}

	if (this->should_unmark)
	{
		// unmark everything at destination location (before it gets changed by observation
		// callback), which should now include the placed object
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

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingPlaceBehavior::get_retained_resources(dhtt::Node *container)
{
	(void)container;
	return std::vector<dhtt_msgs::msg::Resource>{};
}

std::vector<dhtt_msgs::msg::Resource>
CookingPlaceBehavior::get_released_resources(dhtt::Node *container)
{
	return container->get_owned_resources();
}

std::vector<dhtt_msgs::msg::Resource> CookingPlaceBehavior::get_necessary_resources()
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