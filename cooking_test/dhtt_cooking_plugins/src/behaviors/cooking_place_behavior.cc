#include "dhtt_cooking_plugins/behaviors/cooking_place_behavior.hpp"

namespace dhtt_cooking_plugins
{
double CookingPlaceBehavior::get_perceived_efficiency(dhtt::Node* container)
{
	(void) container;

	return 1.0;
}

void CookingPlaceBehavior::do_work(dhtt::Node *container)
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
	
	// check if the object was actually placed in the world
	auto current_holding = this->observed_agent.holding;
	
	if ( last_holding.size() <= current_holding.size() )
	{
		DHTT_LOG_ERROR(this->com_agg, "place did not succeed inventory did not change.");
		
		this->done = false;
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

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingPlaceBehavior::get_retained_resources(dhtt::Node *container)
{
	if ( this->should_keep )
		return container->get_owned_resources();

	// unmark everything at the placed location ( but nothing else duh )
	if ( this->should_unmark )
	{
		std::vector<dhtt_msgs::msg::Resource> not_at_location;

		std::string static_in_front = this->get_static_in_front();
		std::vector<std::string> dynamic_in_front = this->get_all_dynamic_in_front();

		// should unmark a deliver square if that's what we just placed on
		for ( auto iter : container->get_owned_resources() )
			if ( dhtt_cooking_utils::is_cooking_obj(iter.type) and
					iter.name != static_in_front and 
					std::find(dynamic_in_front.begin(), dynamic_in_front.end(), iter.name) == dynamic_in_front.end() )
				not_at_location.push_back(iter);

		return not_at_location;
	}

	auto all = container->get_owned_resources();

	std::vector<dhtt_msgs::msg::Resource> to_ret;
	
	for ( auto resource : all )
		if ( dhtt_cooking_utils::is_cooking_obj(resource.type) )
			to_ret.push_back(resource);

	return to_ret;
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