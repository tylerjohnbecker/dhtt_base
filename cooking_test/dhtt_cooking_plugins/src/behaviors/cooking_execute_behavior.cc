#include "dhtt_cooking_plugins/behaviors/cooking_execute_behavior.hpp"

namespace dhtt_cooking_plugins
{
double CookingExecuteBehavior::get_perceived_efficiency(dhtt::Node* container)
{
	(void) container;
	// this->activation_potential = CookingBehavior::get_perceived_efficiency(container);
	return .7;
}

void CookingExecuteBehavior::do_work(dhtt::Node *container)
{
	const auto prev_dest_obj = this->destination_object;

	/* move_to */
	auto req = std::make_shared<dhtt_cooking_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_cooking_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_cooking_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
	req->action.action_type = dhtt_cooking_msgs::msg::CookingAction::MOVE_TO;

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

	// get objects on square in front of the agent before execution
	auto objects_before = this->get_all_dynamic_in_front();

	/* execute_action */
	req = std::make_shared<dhtt_cooking_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_cooking_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_cooking_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;

	// see pr2.yaml
	req->action.action_type = CookingBehavior::which_arm(container) == "left_arm"
								  ? dhtt_cooking_msgs::msg::CookingAction::EXECUTE_ACTION_ARM1
								  : dhtt_cooking_msgs::msg::CookingAction::EXECUTE_ACTION_ARM2;

	res = this->send_request_and_update(req);

	// get objects on square in front of the agent after execution
	auto objects_after = this->get_all_dynamic_in_front();

	suc = res.get()->success;
	if (not suc)
	{
		DHTT_LOG_ERROR(this->com_agg, "execute_action request did not succeed: " << 
					 res.get()->error_msg);
	}

	// find removed objects
	for ( auto resource_name : objects_before )
	{
		if ( std::find(objects_after.begin(), objects_after.end(), resource_name) == objects_after.end() )
		{
			dhtt_msgs::msg::Resource remove;
			remove.type = dhtt_cooking_utils::resource_name_to_type(resource_name);
			remove.name = resource_name;

			this->remove_resources.push_back(remove);
		}
	}

	// find added objects
	for ( auto resource_name : objects_after )
	{
		if ( std::find(objects_before.begin(), objects_before.end(), resource_name) == objects_before.end() )
		{
			dhtt_msgs::msg::Resource add;
			add.type = dhtt_cooking_utils::resource_name_to_type(resource_name);
			add.name = resource_name;

			this->add_resources.push_back(add);
		}
	}

	// if we are cooking something we want to remove the dynamics now because they haven't changed and will be removed after cooking
	if ( dhtt_cooking_utils::is_cooking_tool(this->observed_env[this->get_static_in_front()]) )
	{
		for ( auto resource_name : objects_after )
		{
			dhtt_msgs::msg::Resource remove;
			remove.type = dhtt_cooking_utils::resource_name_to_type(resource_name);
			remove.name = resource_name;

			this->remove_resources.push_back(remove);
		}
	}

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingExecuteBehavior::get_retained_resources(dhtt::Node *container)
{
	std::vector<dhtt_msgs::msg::Resource> to_ret;
	bool first = true;

	// just keep access to the first gripper found (shouldn't matter for now) and all of the dynamic cooking objects
	for (auto resource_iter : container->get_owned_resources())
	{
		if ( resource_iter.type == dhtt_msgs::msg::Resource::GRIPPER and first)
		{
			to_ret.push_back(resource_iter);

			first = false;
		}
		else if ( dhtt_cooking_utils::is_cooking_obj(resource_iter.type) )
			to_ret.push_back(resource_iter);
	}

	// also retain the added resources
	for ( auto iter : this->add_resources )
		to_ret.push_back(iter);

	// also remove the deleted resources
	dhtt_msgs::msg::Resource look_for;
	auto same_resource = [&]( dhtt_msgs::msg::Resource to_check) { return not strcmp(look_for.name.c_str(), to_check.name.c_str()) ; }; 

	for ( auto iter : this->remove_resources )
	{
		look_for = iter;
		std::vector<dhtt_msgs::msg::Resource>::iterator to_remove;

		while ( ( to_remove = std::find_if(to_ret.begin(), to_ret.end(), same_resource) ) != to_ret.end() )
			to_ret.erase(to_remove);
	}

	return to_ret;
}

std::vector<dhtt_msgs::msg::Resource> CookingExecuteBehavior::get_necessary_resources()
{
	std::vector<dhtt_msgs::msg::Resource> to_ret;

	dhtt_msgs::msg::Resource base;
	base.type = dhtt_msgs::msg::Resource::BASE;

	dhtt_msgs::msg::Resource gripper;
	gripper.type = dhtt_msgs::msg::Resource::GRIPPER;

	for ( auto iter : this->necessary_world_resources )
		to_ret.push_back(iter);

	to_ret.push_back(base);
	to_ret.push_back(gripper);

	return to_ret;
}

} // namespace dhtt_plugins