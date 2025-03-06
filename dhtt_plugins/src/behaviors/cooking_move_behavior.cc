#include <utility>

#include "dhtt_plugins/behaviors/cooking_move_behavior.hpp"

namespace dhtt_plugins
{
CookingMoveBehavior::CookingMoveBehavior() {}

void CookingMoveBehavior::parse_params(std::vector<std::string> params)
{
	// Parse params is executed by ActionType after it creates pub_node_ptr, which we need to create
	// services and such. This should go in the constructor, but pub_node_ptr wouldn't exist yet. It
	// so happens that parse_params is called at the same time we would want an initializer, so it
	// fits the API better.
	this->initialize_();
	auto parse_coord_string = [](const std::string &coord)
	{
		std::istringstream iss(coord); // pretty nifty: skips whitespace
		float num1;
		float num2;
		char comma;
		geometry_msgs::msg::Point point;

		if (!(iss >> num1))
		{
			throw std::invalid_argument("Failed to parse the first number.");
		}

		if (!(iss >> comma) || comma != ',')
		{
			throw std::invalid_argument("Expected a comma between numbers.");
		}

		// Read the second number
		if (!(iss >> num2))
		{
			throw std::invalid_argument("Failed to parse the second number.");
		}

		point.x = num1;
		point.y = num2;
		return point;
	};

	// Copied from move_behavior.cc
	if (static_cast<int>(params.size()) > 2)
		throw std::invalid_argument("Too many parameters passed to node. Only activation "
									"potential, and coord/object required.");

	if (static_cast<int>(params.size()) == 0)
	{
		this->activation_potential = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));

		return;
	}

	auto separator_pos = params[0].find(": ");

	/* Activation Potential */
	if (separator_pos == std::string::npos)
		throw std::invalid_argument(
			"Parameters are expected in the format \"key: value\" but received in the form " +
			params[0] + ". Returning in error.");

	std::string key = params[0].substr(0, separator_pos);
	std::string value = params[0].substr(separator_pos + 2, params[0].size() - separator_pos);

	if (key != this->PARAM_ACTIVATION_POTENTIAL)
		throw std::invalid_argument("Expected parameter " + this->PARAM_ACTIVATION_POTENTIAL +
									", but received " + key + ". Returning in error.");

	this->activation_potential = atof(value.c_str());

	separator_pos = params[1].find(": ");

	if (separator_pos == std::string::npos)
		throw std::invalid_argument(
			"Parameters are expected in the format \"key: value\" but received in the form " +
			params[1] + ". Returning in error.");

	key = params[1].substr(0, separator_pos);
	value = params[1].substr(separator_pos + 2, params[0].size() - separator_pos);

	if (not(key == this->PARAM_COORDINATE or key == this->PARAM_OBJECT_TYPE))
		throw std::invalid_argument("Expected parameter " + this->PARAM_COORDINATE + " or " +
									this->PARAM_OBJECT_TYPE + ", but received " + key +
									". Returning in error.");

	if (key == this->PARAM_COORDINATE)
	{
		this->destination_point = parse_coord_string(value);
	}
	this->destination_type = key;
	this->destination = value;
	this->params = params;
}

double CookingMoveBehavior::get_perceived_efficiency()
{
	// TODO consider implementing walking_distance in cooking zoo for this
	auto point_distance =
		[](const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2)
	{ return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2)); };

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

	// At this point, we've checked that there is a valid observation, and
	// set_destination_to_closest_object() should have been called in observation_callback(), so
	// agent_loc and destination_point should be good to go.

	// assume there is only one agent
	geometry_msgs::msg::Point agent_loc = this->last_obs->agents.front().object_members.location;

	double to_ret = pow(point_distance(agent_loc, this->destination_point), -1);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "I report %f efficiency", to_ret);
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

void CookingMoveBehavior::initialize_()
{
	this->cooking_observation_subscriber =
		this->pub_node_ptr->create_subscription<dhtt_msgs::msg::CookingObservation>(
			"Cooking_Observations", 10,
			std::bind(&CookingMoveBehavior::observation_callback, this, std::placeholders::_1));

	this->cooking_request_client =
		this->pub_node_ptr->create_client<dhtt_msgs::srv::CookingRequest>("Cooking_Server", 10);

	if (not this->cooking_request_client->wait_for_service(std::chrono::seconds(1)))
	{
		RCLCPP_FATAL(this->pub_node_ptr->get_logger(), "Could not contact dhtt_cooking service");
		throw std::runtime_error("Could not contact dhtt_cooking service");
	}

	// Make sure we get an initial observation
	auto req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
	req->super_action = dhtt_msgs::srv::CookingRequest::Request::OBSERVE;
	auto res = this->cooking_request_client->async_send_request(req);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "Sent initial observation request");
	this->executor->spin_until_future_complete(res);
	RCLCPP_INFO(this->pub_node_ptr->get_logger(),
				"Received initial observation request: We're good to go.");

	if (not this->last_obs)
	{
		RCLCPP_FATAL(this->pub_node_ptr->get_logger(),
					 "Did not get first observation. This node may not behave properly.");
	}
}

void CookingMoveBehavior::observation_callback(
	std::shared_ptr<dhtt_msgs::msg::CookingObservation> msg)
{
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "Inside observation callback");
	this->last_obs = std::move(msg);
	if (this->destination_type == this->PARAM_OBJECT_TYPE)
	{
		this->set_destination_to_closest_object();
	}
}

void CookingMoveBehavior::set_destination_to_closest_object()
{
	// TODO use this
	// Assumes on entry that this->last_obs is valid. I.e. we've received an observation and
	// destination_type is object not coord. It is the callers responsibility to be sure of that,
	// otherwise this function doesn't do anything.
	const auto found_iter =
		std::find(this->last_obs->closest_objects_types.cbegin(),
				  this->last_obs->closest_objects_types.cend(), this->destination);
	if (found_iter == this->last_obs->closest_objects_types.cend())
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 ("Could not find object of type: " + this->destination).c_str());
		return;
	}

	const auto index = std::distance(this->last_obs->closest_objects_types.cbegin(), found_iter);
	this->destination_point = this->last_obs->closest_objects_locations[index];
}

} // namespace dhtt_plugins