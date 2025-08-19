#include "dhtt_cooking_plugins/behaviors/cooking_pick_behavior.hpp"

namespace dhtt_cooking_plugins
{
double CookingPickBehavior::get_perceived_efficiency(dhtt::Node* container)
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

	return 0.8;
}

void CookingPickBehavior::do_work(dhtt::Node *container)
{
	(void)container; // Unused

	std::vector<dhtt_cooking_msgs::msg::CookingObject> prev_held = this->last_obs->agents[0].holding;
	auto last_holding = this->last_obs->agents[0].holding;

	// Equivalent of get_released_resources() but for "resources" on the paramserver
	DHTT_LOG_INFO(this->com_agg, 
				"Unmarking object that was under " <<
				 this->destination_object.object_type
					);
	if (this->should_unmark and not this->unmark_static_object_under_obj(this->destination_object, true))
	{
		DHTT_LOG_WARN(this->com_agg, 
					"Error unmarking static object under " << this->destination_object.object_type);
	}

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

	auto current_holding = this->last_obs->agents[0].holding;
	
	if ( last_holding.size() >= current_holding.size() )
	{
		DHTT_LOG_ERROR(this->com_agg, "pick did not succeed inventory did not change.");
		
		this->done = false;
		return;
	}

	// Mark the object that was picked. If the static object had multiple objects on top (chopping
	// bread), it is ambiguous which one will be picked.
	dhtt_cooking_msgs::msg::CookingObject picked_obj;
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
		DHTT_LOG_INFO(this->com_agg, "Marking object as " << this->destination_mark);
		suc = this->mark_object(picked_obj.world_id, this->destination_mark);
		if (not suc)
		{
			DHTT_LOG_ERROR(this->com_agg,  "Marking object failed: " <<
						 res.get()->error_msg);
		}
	}

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingPickBehavior::get_retained_resources(dhtt::Node *container)
{
	if (this->should_keep)
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
	if (this->should_keep)
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