#include "dhtt_plugins/behaviors/cooking_object_exists_behavior.hpp"

namespace dhtt_plugins
{
double CookingObjectExistsBehavior::get_perceived_efficiency(dhtt::Node* container)
{
	this->activation_potential = CookingBehavior::get_perceived_efficiency(container);

	// TODO dbl_epsilon
	if (this->activation_potential < .0001 and this->should_unmark)
		this->activation_potential = .0001;

	return this->activation_potential;
}

void CookingObjectExistsBehavior::do_work(dhtt::Node *container)
{
	(void)container;
	this->done |= this->destination_is_good; // TODO check this and if we need to do can_work

	// Tick the world with a NOP
	auto req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
	req->action.action_type = dhtt_msgs::msg::CookingAction::NO_OP;

	if (this->destination_is_good)
	{
		if (not this->mark_object(this->destination_object.world_id, this->destination_mark))
		{
			RCLCPP_ERROR_STREAM(container->get_logger(),
								"Marking object " << this->destination_object.world_id << " with "
												  << this->destination_mark << " failed.");
			this->done = false;
		}
	}

	auto res = this->send_request_and_update(req);
	// auto res = this->cooking_request_client->async_send_request(req);
	// RCLCPP_INFO(container->get_logger(), "Sending nop request");
	// this->com_agg->spin_until_future_complete<std::shared_ptr<dhtt_msgs::srv::CookingRequest::Response>>(res);
	// RCLCPP_INFO(container->get_logger(), "nop request completed");

	bool suc = res.get()->success;
	if (not suc)
	{
		RCLCPP_ERROR(container->get_logger(), "nop request did not succeed: %s",
					 res.get()->error_msg.c_str());
	}

	this->done &= suc;
}

std::vector<dhtt_msgs::msg::Resource>
CookingObjectExistsBehavior::get_retained_resources(dhtt::Node *container)
{
	(void)container;
	return {};
}
std::vector<dhtt_msgs::msg::Resource>
CookingObjectExistsBehavior::get_released_resources(dhtt::Node *container)
{
	return container->get_owned_resources();
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