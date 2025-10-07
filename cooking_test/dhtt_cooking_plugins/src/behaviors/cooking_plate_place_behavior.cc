#include "dhtt_cooking_plugins/behaviors/cooking_plate_place_behavior.hpp"

namespace dhtt_cooking_plugins
{
void CookingPlatePlaceBehavior::do_work(dhtt::Node *container)
{
	(void)container; // Unused

	const auto prev_dest_obj = this->destination_object;
	auto last_holding = this->observed_agent.holding;

	/* interact_primary */
	auto req = std::make_shared<dhtt_cooking_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_cooking_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_cooking_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

	// see pr2.yaml
	req->action.action_type = CookingBehavior::which_arm(container) == "left_arm"
								  ? dhtt_cooking_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM1
								  : dhtt_cooking_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM2;

	auto res = this->send_request_and_update(req);

	bool suc = res.get()->success;
	if (not suc)
	{
		DHTT_LOG_ERROR(this->com_agg, 
					 "interact_primary request did not succeed: " << res.get()->error_msg);
		return;
	}

	/* if placing on a deliversquare, send an extra nop so the deliversquare clears and unmark it if
	 * needed*/
	if (this->destination_value.find("Deliversquare") != std::string::npos)
	{
		auto remove_in_front = this->get_all_dynamic_in_front();

		req = std::make_shared<dhtt_cooking_msgs::srv::CookingRequest::Request>();
		req->super_action = dhtt_cooking_msgs::srv::CookingRequest::Request::ACTION;
		req->action.player_name = dhtt_cooking_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
		req->action.action_type = dhtt_cooking_msgs::msg::CookingAction::NO_OP;

		// then send the NOP which should delete the objects on the deliver square
		res = this->send_request_and_update(req);

		suc = res.get()->success;

		if (not suc)
		{
			DHTT_LOG_ERROR(this->com_agg, "NOP request did not succeed: " <<
						 res.get()->error_msg);
			return;
		}

		// remove any resources that were delivered and deleted
		for ( auto iter : remove_in_front )
		{
			dhtt_msgs::msg::Resource to_remove;
			to_remove.name = iter;

			this->remove_resources.push_back(to_remove);
		}
		
	}
	
	/* second interact_primary */
	req = std::make_shared<dhtt_cooking_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_cooking_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_cooking_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

	// see pr2.yaml
	req->action.action_type = CookingBehavior::which_arm(container) == "left_arm"
								  ? dhtt_cooking_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM1
								  : dhtt_cooking_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM2;

	res = this->send_request_and_update(req);

	suc = res.get()->success;
	if (not suc)
	{
		DHTT_LOG_ERROR(this->com_agg, 
					 "interact_primary request did not succeed: " << res.get()->error_msg);
		return;
	}
	
	// check if the object was actually placed in the world
	auto current_holding = this->observed_agent.holding;
	
	if ( last_holding.size() <= current_holding.size() )
	{
		DHTT_LOG_ERROR(this->com_agg, "place did not succeed inventory did not change.");
		
		this->done = false;
		return;
	}

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource> CookingPlatePlaceBehavior::get_retained_resources(dhtt::Node *container) 
{
	if ( this->should_unmark )
		return std::vector<dhtt_msgs::msg::Resource>();

	return CookingPlaceBehavior::get_retained_resources(container);
}
} // namespace dhtt_plugins