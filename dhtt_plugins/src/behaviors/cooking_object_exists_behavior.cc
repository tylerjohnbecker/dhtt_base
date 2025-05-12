#include "dhtt_plugins/behaviors/cooking_object_exists_behavior.hpp"

namespace dhtt_plugins
{
double CookingObjectExistsBehavior::get_perceived_efficiency()
{
	return CookingBehavior::get_perceived_efficiency();
}

void CookingObjectExistsBehavior::do_work(dhtt::Node *container)
{
	(void) container;
	this->done |= this->destination_is_good; // TODO check this and if we need to do can_work

	// Tick the world with a NOP
	auto req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
	req->action.action_type = dhtt_msgs::msg::CookingAction::NO_OP;

	auto res = this->cooking_request_client->async_send_request(req);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "Sending move_to request");
	this->executor->spin_until_future_complete(res);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "move_to request completed");

	bool suc = res.get()->success;
	if (not suc)
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(), "move_to request did not succeed: %s",
					 res.get()->error_msg.c_str());
	}
}

std::vector<dhtt_msgs::msg::Resource>
CookingObjectExistsBehavior::get_retained_resources(dhtt::Node *container)
{
	return container->get_owned_resources();
}
std::vector<dhtt_msgs::msg::Resource>
CookingObjectExistsBehavior::get_released_resources(dhtt::Node *container)
{
	(void) container;
	return {};
}

std::vector<dhtt_msgs::msg::Resource> CookingObjectExistsBehavior::get_necessary_resources()
{
	return {};
}
} // namespace dhtt_plugins