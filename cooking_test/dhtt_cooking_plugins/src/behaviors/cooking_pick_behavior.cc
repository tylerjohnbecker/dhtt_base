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

	std::vector<dhtt_cooking_msgs::msg::CookingObject> prev_held = this->observed_agent.holding;
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

	auto current_holding = this->observed_agent.holding;
	
	if ( last_holding.size() >= current_holding.size() )
	{
		DHTT_LOG_ERROR(this->com_agg, "pick did not succeed inventory did not change.");
		
		this->done = false;
		return;
	}

	// Mark the object that was picked. If the static object had multiple objects on top (chopping
	// bread), it is ambiguous which one will be picked.
	dhtt_cooking_msgs::msg::CookingObject picked_obj;
	for (const auto &held : this->observed_agent.holding)
	{
		if (std::find(prev_held.cbegin(), prev_held.cend(), held) == prev_held.cend())
		{
			picked_obj = held;
			break; // assume we can only pick one object
		}
	}

	std::string held_resource_name = dhtt_cooking_utils::get_resource_name(picked_obj);

	DHTT_LOG_INFO(this->com_agg, "Aquired object: " << held_resource_name );

	// picked_resource msg
	dhtt_msgs::msg::Resource picked_resource;
	picked_resource.type = dhtt_cooking_utils::resource_name_to_type(held_resource_name);
	picked_resource.name = held_resource_name;

	this->necessary_world_resources.push_back(picked_resource);

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
	bool first = true;

	std::string front_static = this->get_static_in_front();

	// just keep access to the first gripper found (shouldn't matter for now) and all of the cooking objects other than the one in front
	for (auto resource_iter : container->get_owned_resources())
	{
		if ( resource_iter.type == dhtt_msgs::msg::Resource::GRIPPER and first)
		{
			to_ret.push_back(resource_iter);

			first = false;
		}
		else if ( dhtt_cooking_utils::is_cooking_obj(resource_iter.type) and strcmp(resource_iter.name.c_str(), front_static.c_str()) )
			to_ret.push_back(resource_iter);
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