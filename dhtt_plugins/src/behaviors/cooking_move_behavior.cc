#include <utility>

#include "dhtt_plugins/behaviors/cooking_move_behavior.hpp"

namespace dhtt_plugins
{
double CookingMoveBehavior::get_perceived_efficiency()
{
	// Make sure we're up to date with observations
	this->executor->spin_all(std::chrono::nanoseconds(0));

	if (not this->last_obs)
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "No observation yet. This node probably won't behave as intended. Setting "
					 "activation_potential to 0");
		return 0;
	}

	if (this->destination_point.x == 0 and this->destination_point.y == 0)
	{
		RCLCPP_WARN(this->pub_node_ptr->get_logger(),
					"Destination location is (0,0), are you sure this is correct?");
	}

	if (this->last_obs->agents.empty())
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "No agents in observation yet. Setting activation_potential to 0");
		return 0;
	}

	if (not this->destination_is_good)
	{
		RCLCPP_WARN(this->pub_node_ptr->get_logger(),
					"Don't have a good destination (is the object in the right state?). Setting "
					"activation_potential to 0.");
		return 0;
	}

	// At this point, we've checked that there is a valid observation, and
	// set_destination_to_closest_object() should have been called in observation_callback(), so
	// agent_loc and destination_point should be good to go.

	double to_ret = pow(1.0 + this->agent_point_distance(this->destination_point), -1);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "I report %f efficiency", to_ret);

	this->activation_potential = to_ret; // TODO is this necessary?
	return to_ret;
}

void CookingMoveBehavior::do_work(dhtt::Node *container)
{
	(void)container; // Unused

	auto req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::ACTION;
	req->action.player_name = dhtt_msgs::msg::CookingAction::DEFAULT_PLAYER_NAME;
	req->action.action_type = dhtt_msgs::msg::CookingAction::MOVE_TO;

	std::string dest_point_str = std::to_string(this->destination_point.x) + ", " +
								 std::to_string(this->destination_point.y);

	req->action.params = dest_point_str;

	auto res = this->cooking_request_client->async_send_request(req);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "Sending move_to request");
	this->executor->spin_until_future_complete(res);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "move_to request completed");
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

} // namespace dhtt_plugins