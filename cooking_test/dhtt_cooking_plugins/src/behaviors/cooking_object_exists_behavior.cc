#include "dhtt_cooking_plugins/behaviors/cooking_object_exists_behavior.hpp"

namespace dhtt_cooking_plugins
{
double CookingObjectExistsBehavior::get_perceived_efficiency(dhtt::Node* container)
{

	int8_t type_to_request = dhtt_cooking_utils::param_to_msg_val(this->destination_value + this->destination_conditions);

	if ( strcmp( "", this->has_resource_free_of_type(container, type_to_request).c_str() ) ) 
		return 1.0;
	
	return .0001;
}

void CookingObjectExistsBehavior::do_work(dhtt::Node *container)
{
	(void)container;
	this->done |= this->destination_is_good; // TODO check this and if we need to do can_work

	// Tick the world with a NOP
	auto req = std::make_shared<dhtt_cooking_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_cooking_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_cooking_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
	req->action.action_type = dhtt_cooking_msgs::msg::CookingAction::NO_OP;

	auto res = this->send_request_and_update(req);

	// now check if the object exists in the world in order to try and claim it
	int8_t type_to_request = dhtt_cooking_utils::param_to_msg_val(this->destination_value + this->destination_conditions);
	std::string found = "";

	if ( strcmp( "", ( found = this->has_resource_free_of_type(container, type_to_request) ).c_str() ) )
	{
		this->done = true;

		dhtt_msgs::msg::Resource exists_request;
		exists_request.type = type_to_request;
		exists_request.name = found;

		this->necessary_world_resources.push_back(exists_request);

		dhtt_msgs::msg::Resource counter_underneath;
		counter_underneath.name = this->get_static_underneath(found);

		this->necessary_world_resources.push_back(counter_underneath);
	}
}

std::vector<dhtt_msgs::msg::Resource>
CookingObjectExistsBehavior::get_retained_resources(dhtt::Node *container)
{
	std::vector<dhtt_msgs::msg::Resource> to_ret;
	bool first = true;

	// just keep access to the first gripper found (shouldn't matter for now) and all of the cooking objects
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

	return to_ret;
}

std::vector<dhtt_msgs::msg::Resource> CookingObjectExistsBehavior::get_necessary_resources()
{
	std::vector<dhtt_msgs::msg::Resource> to_ret;

	dhtt_msgs::msg::Resource base;
	base.type = dhtt_msgs::msg::Resource::BASE;

	to_ret.push_back(base);

	return to_ret;
}
} // namespace dhtt_plugins