#include "dhtt_plugins/behaviors/cooking_behavior.hpp"

namespace dhtt_plugins
{

void CookingBehavior::parse_params(std::vector<std::string> params)
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
		throw std::invalid_argument(
			"Too many parameters passed to node. Coord or object required and "
			"optionally object conditions.");

	if (static_cast<int>(params.size()) == 0)
	{
		throw std::invalid_argument(
			"Not enough parameters passed to node. Coord or Object required and "
			"optionally object conditions.");
	}

	/* Coord/Object type */
	auto separator_pos = params[0].find(": ");
	if (separator_pos == std::string::npos)
		throw std::invalid_argument(
			"Parameters are expected in the format \"key: value\" but received in the form " +
			params[0] + ". Returning in error.");

	std::string key = params[0].substr(0, separator_pos);
	std::string value = params[0].substr(separator_pos + 2, params[0].size() - separator_pos);

	if (not(key == this->PARAM_COORDINATE or key == this->PARAM_OBJECT_TYPE))
		throw std::invalid_argument("Expected parameter " + this->PARAM_COORDINATE + " or " +
									this->PARAM_OBJECT_TYPE + ", but received " + key +
									". Returning in error.");
	this->destination_type = key;
	this->destination_value = value;

	if (this->destination_type == this->PARAM_COORDINATE)
	{
		this->destination_point = parse_coord_string(this->destination_value);
		this->destination_is_good = true;
	}
	else if (this->destination_type == this->PARAM_OBJECT_TYPE)
	{
		if (static_cast<int>(params.size()) != 2)
		{
			this->destination_conditions = "";
		}
		else
		{
			/* Object Conditions */
			separator_pos = params[1].find(": ");
			if (separator_pos == std::string::npos)
				throw std::invalid_argument("Parameters are expected in the format \"key: value\" "
											"but received in the form " +
											params[1] + ". Returning in error.");
			key = params[1].substr(0, separator_pos);
			value = params[1].substr(separator_pos + 2, params[0].size() - separator_pos);

			if (key != this->PARAM_OBJECT_CONDITIONS)
			{
				throw std::invalid_argument("Expected parameter " + this->PARAM_OBJECT_CONDITIONS +
											", but received " + key);
			}

			this->destination_conditions = value;
		}

		this->set_destination_to_closest_object();
	}
	else
	{
		throw std::invalid_argument("");
	}

	this->params = params;
}

double CookingBehavior::get_perceived_efficiency()
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

	return 1; // return something other than 0

	// At this point, we've checked that there is a valid observation, and
	// set_destination_to_closest_object() should have been called in observation_callback(), so
	// agent_loc and destination_point should be good to go.

	// return your measure here
}

double CookingBehavior::point_distance(const geometry_msgs::msg::Point &point1,
									   const geometry_msgs::msg::Point &point2)
{
	return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

double CookingBehavior::agent_point_distance(const geometry_msgs::msg::Point &point) const
{
	// Assumes we have a valid observation
	// assume there is only one agent
	geometry_msgs::msg::Point agent_loc = this->last_obs->agents.front().object_members.location;
	return this->point_distance(agent_loc, point);
}

void CookingBehavior::initialize_()
{
	this->cooking_observation_subscriber =
		this->pub_node_ptr->create_subscription<dhtt_msgs::msg::CookingObservation>(
			"Cooking_Observations", 10,
			std::bind(&CookingBehavior::observation_callback, this, std::placeholders::_1));

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

void CookingBehavior::observation_callback(std::shared_ptr<dhtt_msgs::msg::CookingObservation> msg)
{
	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "Inside observation callback");
	this->last_obs = std::move(msg);

	// Chicken-egg, This may be called before parse_params() has set destination_* members, which
	// set_destination_to_closest_object() needs to find the relevant object, but that function also
	// needs this initial observation. Check that they are set.
	if (this->destination_type == this->PARAM_OBJECT_TYPE and
		not this->destination_value.empty() and not this->destination_conditions.empty())
	{
		this->set_destination_to_closest_object();
	}
}

void CookingBehavior::set_destination_to_closest_object()
{
	// Assumes on entry that this->last_obs is valid. I.e. we've received an observation and
	// destination_type is object not coord. It is the callers responsibility to be sure of that,
	auto &dst_val = this->destination_value;
	auto &dst_conds = this->destination_conditions;

	RCLCPP_INFO(this->pub_node_ptr->get_logger(), "Setting destination to val: %s, with conds: %s",
				dst_val.c_str(), dst_conds.c_str());

	if (dst_val.empty())
	{
		throw std::runtime_error("Empty destination_value");
	}

	// Helper to trim whitespace from beginning and end of a string.
	auto trim = [](const std::string &str)
	{
		auto start = str.begin();
		while (start != str.end() && std::isspace(*start))
		{
			++start;
		}

		auto end = str.end();
		do
		{
			--end;
		} while (std::distance(start, end) > 0 && std::isspace(*end));

		return std::string(start, end + 1);
	};

	// Helper to parse comma-separated conditions into a vector of conditions
	auto parse_coord_string = [trim](const std::string &conds_str)
	{
		std::istringstream iss(conds_str); // pretty nifty: skips whitespace
		std::vector<std::string> conds;
		std::string cond;

		while (std::getline(iss, cond, ','))
		{
			conds.push_back(trim(cond));
		}

		return conds;
	};
	std::vector<std::string> conds = parse_coord_string(dst_conds);

	auto pred = [dst_val, conds](dhtt_msgs::msg::CookingObject obj)
	{
		if (obj.object_type == dst_val)
		{
			// Note that if conds is empty (user provided empty string ""), we select the closest of
			// any condition.
			for (auto cond : conds)
			{
				if (std::find(obj.physical_state.begin(), obj.physical_state.end(), cond) ==
					obj.physical_state.end())
				{
					return false; // Not the right condition
				}
			}
			return true; // Correct type and conditions
		}
		return false; // Not correct type
	};

	geometry_msgs::msg::Point ret;
	const auto found_iter =
		std::find_if(this->last_obs->objects.cbegin(), this->last_obs->objects.cend(), pred);

	if (found_iter == this->last_obs->objects.cend())
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 ("Could not find object of type: " + dst_val).c_str());
		this->destination_is_good = false;
	}
	else
	{
		ret = found_iter->location;

		// See if we can't find a closer object that matches
		for (auto iter = found_iter + 1; iter != this->last_obs->objects.cend(); ++iter)
		{
			if (pred(*iter) and
				this->agent_point_distance(iter->location) < this->agent_point_distance(ret))
			{
				ret = iter->location;
			}
		}
		this->destination_is_good = true;
		this->destination_point = ret;
	}
}

} // namespace dhtt_plugins