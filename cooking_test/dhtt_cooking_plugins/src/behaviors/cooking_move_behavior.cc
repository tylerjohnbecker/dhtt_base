#include "dhtt_cooking_plugins/behaviors/cooking_move_behavior.hpp"

namespace dhtt_cooking_plugins
{
double CookingMoveBehavior::get_perceived_efficiency(dhtt::Node* container)
{
	double activation_check = CookingBehavior::get_perceived_efficiency(container);

	if (activation_check != 0)
	{
		double to_ret = pow(1.0 + this->agent_point_distance(this->destination_point), -1);

		this->activation_potential = to_ret; // TODO is this necessary?
		return to_ret;
	}

	return 0;
}

void CookingMoveBehavior::do_work(dhtt::Node *container)
{
	(void)container; // Unused

	// if (not CookingBehavior::can_work())
	// {
	// 	return;
	// }

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
		DHTT_LOG_ERROR(this->com_agg, "move_to request did not succeed: " <<
					 res.get()->error_msg);
		return;
	}

	// if object is not marked for anyone
	// if (not this->destination_mark.empty() and this->check_mark(this->destination_object) == '2')
	// {
	// 	DHTT_LOG_INFO(this->com_agg, 
	// 				"Marking object as " << this->destination_mark);
	// 	suc = this->mark_object(this->destination_object.world_id, this->destination_mark);
	// 	if (not suc)
	// 	{
	// 		DHTT_LOG_ERROR(this->com_agg, "Marking object failed: " <<
	// 					 res.get()->error_msg);
	// 	}
	// }

	this->done = suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingMoveBehavior::get_retained_resources(dhtt::Node *container)
{
	return container->get_owned_resources();
}

std::vector<dhtt_msgs::msg::Resource>
CookingMoveBehavior::get_released_resources(dhtt::Node *container)
{
	(void)container; // unused
	return {};
}
std::vector<dhtt_msgs::msg::Resource> CookingMoveBehavior::get_necessary_resources()
{
	std::vector<dhtt_msgs::msg::Resource> to_ret;

	dhtt_msgs::msg::Resource base;
	base.type = dhtt_msgs::msg::Resource::BASE;

	// TODO sometimes seems to help
	dhtt_msgs::msg::Resource gripper;
	gripper.type = dhtt_msgs::msg::Resource::GRIPPER;

	to_ret.push_back(base);
	to_ret.push_back(gripper);

	return to_ret;
}

void CookingMoveBehavior::populate_resource_lists (dhtt::Node* container)
{
	// empty the list so it doesn't contain junk
	this->necessary_world_resources.clear();

	if (this->observed_env.size() == 0)
	{
		DHTT_LOG_DEBUG(this->com_agg,
					"No observation yet. This node probably won't behave as intended. Making " << 
					"Observe request");

		// Make sure we get an initial observation
		auto req = std::make_shared<dhtt_cooking_msgs::srv::CookingRequest::Request>();
		req->super_action = dhtt_cooking_msgs::srv::CookingRequest::Request::OBSERVE;
		auto res = this->send_request_and_update(req);
		// auto res = this->cooking_request_client->async_send_request(req);
		// this->com_agg->spin_until_future_complete<std::shared_ptr<dhtt_cooking_msgs::srv::CookingRequest::Response>>(res, std::chrono::seconds(1));

		if (res.valid())
		{
			DHTT_LOG_DEBUG(this->com_agg,
						 "Received initial observation request: We're good to go.");
		}
		else
		{
			DHTT_LOG_ERROR(this->com_agg, "BUG, observation request timed out");
		}

		// no return
	}

	dhtt_msgs::msg::Resource n_world_resource;
	if ( this->destination_conditions != "Free" )
		n_world_resource.type = dhtt_cooking_utils::param_to_msg_val(this->destination_value + this->destination_conditions); // doesn't work if I found a food item for instance
	else
		n_world_resource.type = dhtt_cooking_utils::param_to_msg_val(this->destination_value);

	// choose the name of the closest resource with the given type
	if ( dhtt_cooking_utils::is_static_obj(n_world_resource.type) )
	{
		if ( this->destination_conditions == "Free" )
		{
			n_world_resource.name = this->set_destination_to_closest_free(n_world_resource.type, container->get_resource_state());
		}
		else
		{
			n_world_resource.name = this->set_destination_to_closest(n_world_resource.type, container->get_resource_state());
		}

		// if the name is blank it might be because we own the resource we searched for
		if ( n_world_resource.name == "" )
			this->set_destination_to_closest_owned(n_world_resource.type, container->get_owned_resources());
	}
	else if ( dhtt_cooking_utils::is_dynamic_obj(n_world_resource.type) ) // if it's a dynamic object then we must already own it
	{
		n_world_resource.name = this->set_destination_to_static_under(n_world_resource.type, container->get_owned_resources());
		n_world_resource.type = dhtt_cooking_utils::resource_name_to_type(n_world_resource.name);
	}

	this->necessary_world_resources.push_back(n_world_resource);
}

} // namespace dhtt_plugins