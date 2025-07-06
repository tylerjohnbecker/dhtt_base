#include "dhtt_plugins/behaviors/cooking_plate_place_behavior.hpp"

namespace dhtt_plugins
{
void CookingPlatePlaceBehavior::do_work(dhtt::Node *container)
{
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

	/* first interact_primary */
	req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

	// see pr2.yaml
	req->action.action_type = CookingBehavior::which_arm(container) == "left_arm"
								  ? dhtt_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM1
								  : dhtt_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM2;

	res = this->send_request_and_update(req);
	// res = this->cooking_request_client->async_send_request(req);
	// RCLCPP_INFO(container->get_logger(), "Sending first interact request");
	// this->com_agg->spin_until_future_complete<std::shared_ptr<dhtt_msgs::srv::CookingRequest::Response>>(res);
	// RCLCPP_INFO(container->get_logger(), "First interact completed");

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
		// make sure we're up to date with observations
		// this->com_agg->spin_some();

		// unmark everything on the deliversquare, including the deliversquare
		// need to do this before the objects are deleted by the nop...
		for (auto &world_obj : this->last_obs->objects)
		{
			// TODO if object type is lettuce or tomato or something, debug point

			if (world_obj.location == this->destination_object.location)
			{
				if (not CookingBehavior::unmark_object(world_obj.world_id))
				{
					RCLCPP_ERROR_STREAM(container->get_logger(),
										"Error unmarking object" << world_obj.world_id);
				}
			}
		}

		req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
		req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
		req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

		// see pr2.yaml
		req->action.action_type = dhtt_msgs::msg::CookingAction::NO_OP;

		res = this->send_request_and_update(req);
		// res = this->cooking_request_client->async_send_request(req);
		// RCLCPP_INFO(container->get_logger(), "Sending NOP request");
		// this->com_agg->spin_until_future_complete<std::shared_ptr<dhtt_msgs::srv::CookingRequest::Response>>(res);
		// RCLCPP_INFO(container->get_logger(), "NOP interact completed");

		suc = res.get()->success;
		if (not suc)
		{
			RCLCPP_ERROR(container->get_logger(), "NOP request did not succeed: %s",
						 res.get()->error_msg.c_str());
			return;
		}
	}

	/* second interact_primary */
	req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

	// see pr2.yaml
	req->action.action_type = CookingBehavior::which_arm(container) == "left_arm"
								  ? dhtt_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM1
								  : dhtt_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM2;

	res = this->send_request_and_update(req);
	// res = this->cooking_request_client->async_send_request(req);
	// RCLCPP_INFO(container->get_logger(), "Sending second interact request");
	// this->com_agg->spin_until_future_complete<std::shared_ptr<dhtt_msgs::srv::CookingRequest::Response>>(res);
	// RCLCPP_INFO(container->get_logger(), "Second interact completed");

	suc = res.get()->success;
	if (not suc)
	{
		RCLCPP_ERROR(container->get_logger(),
					 "interact_primary request did not succeed: %s", res.get()->error_msg.c_str());
		return;
	}

	this->done = suc;
}
} // namespace dhtt_plugins