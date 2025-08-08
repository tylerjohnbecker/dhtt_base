#include "dhtt_plugins/behaviors/cooking_plate_place_behavior.hpp"

namespace dhtt_plugins
{
void CookingPlatePlaceBehavior::do_work(dhtt::Node *container)
{
	// if (not CookingBehavior::can_work())
	// {
	// 	return;
	// }
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

	bool suc = res.get()->success;
	if (not suc)
	{
		DHTT_LOG_ERROR(this->com_agg, 
					 "move_to request did not succeed, returning early: " <<
					 res.get()->error_msg);
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

	suc = res.get()->success;
	if (not suc)
	{
		DHTT_LOG_ERROR(this->com_agg,
					 "interact_primary request did not succeed: " << res.get()->error_msg);
		return;
	}

	/* if placing on a deliversquare, send an extra nop so the deliversquare clears and unmark it if
	 * needed*/
	if (this->destination_object.object_type.find("Deliversquare") != std::string::npos)
	{
		req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
		req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
		req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
		req->action.action_type = dhtt_msgs::msg::CookingAction::NO_OP;

		// First unmark everything on the deliver square
		this->unmark_at_location(prev_dest_obj.location);

		// then send the NOP which should delete the objects on the deliver square
		res = this->send_request_and_update(req);

		suc = res.get()->success;

		// this->unmark_all_nonexistant();

		if (not suc)
		{
			DHTT_LOG_ERROR(this->com_agg, "NOP request did not succeed: " << 
						 res.get()->error_msg);
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

	suc = res.get()->success;
	if (not suc)
	{
		DHTT_LOG_ERROR(this->com_agg, 
					 "interact_primary request did not succeed: " << res.get()->error_msg);
		return;
	}

	this->done = suc;
}
} // namespace dhtt_plugins