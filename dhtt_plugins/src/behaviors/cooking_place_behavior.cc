#include "dhtt_plugins/behaviors/cooking_place_behavior.hpp"

namespace dhtt_plugins
{
double CookingPlaceBehavior::get_perceived_efficiency(dhtt::Node* container)
{
	// double activation_check = CookingBehavior::get_perceived_efficiency(container);

	// if (activation_check != 0)
	// {
	// 	// High activation if we're right next to the object. No activation otherwise.
	// 	double to_ret = abs(this->agent_point_distance(this->destination_point) - 1.0) < DBL_EPSILON ? 1.0 : 0.0;

	// 	this->activation_potential = to_ret; // TODO is this necessary?
	// 	return to_ret;
	// }

	(void) container;

	return 1.0;
}

void CookingPlaceBehavior::do_work(dhtt::Node *container)
{
	(void)container; // Unused

	// if (not CookingBehavior::can_work())
	// {
	// 	return;
	// }

	const auto prev_dest_obj = this->destination_object;
	auto last_holding = this->last_obs->agents[0].holding;

	/* interact_primary */
	auto req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

	// see pr2.yaml
	req->action.action_type = CookingBehavior::which_arm(container) == "left_arm"
								  ? dhtt_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM1
								  : dhtt_msgs::msg::CookingAction::INTERACT_PRIMARY_ARM2;

	auto res = this->send_request_and_update(req);

	bool suc = res.get()->success;
	if (not suc)
	{
		DHTT_LOG_ERROR(this->com_agg, 
					 "interact_primary request did not succeed: " << res.get()->error_msg);
		return;
	}
	
	// check if the object was actually placed in the world
	auto current_holding = this->last_obs->agents[0].holding;
	
	if ( last_holding.size() <= current_holding.size() )
	{
		DHTT_LOG_ERROR(this->com_agg, "place did not succeed inventory did not change.");
		
		this->done = false;
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

		this->unmark_all_nonexistant();

		if (not suc)
		{
			DHTT_LOG_ERROR(this->com_agg, "NOP request did not succeed: " <<
						 res.get()->error_msg);
			return;
		}
	}

	if (this->should_unmark)
	{
		// unmark everything at destination location (before it gets changed by observation
		// callback), which should now include the placed object

		this->unmark_at_location(prev_dest_obj.location);
	}

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingPlaceBehavior::get_retained_resources(dhtt::Node *container)
{
	if ( this->should_keep )
		return container->get_owned_resources();

	(void)container;
	return std::vector<dhtt_msgs::msg::Resource>{};
}

std::vector<dhtt_msgs::msg::Resource>
CookingPlaceBehavior::get_released_resources(dhtt::Node *container)
{
	if ( this->should_keep )
		return std::vector<dhtt_msgs::msg::Resource>{};

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