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
			value = params[1].substr(separator_pos + 2, params[1].size() - separator_pos);

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
	if (this->destination_type == this->PARAM_OBJECT_TYPE and not this->destination_value.empty())
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

	std::vector<std::string> conds = CookingBehavior::parse_conds_string(dst_conds);

	auto pred = [dst_val, conds, this](dhtt_msgs::msg::CookingObject obj)
	{
		if (obj.object_type == dst_val)
		{
			// Note that if conds is empty (user provided empty string ""), we select the closest of
			// any condition.
			for (auto cond : conds)
			{
				if (cond == "Free" or cond == "Contains")
				{
					// Counters are not ContentObjects
					if (cond == "Free" and obj.object_type == "Counter")
					{
						// Check counter isn't an unreachable corner
						if ((obj.location.x == 0 and obj.location.y == 0) or
							(obj.location.x == 0 and
							 obj.location.y == CookingBehavior::LEVEL_SIZE) or
							(obj.location.x == CookingBehavior::LEVEL_SIZE and
							 obj.location.y == 0) or
							(obj.location.x == CookingBehavior::LEVEL_SIZE and
							 obj.location.y == CookingBehavior::LEVEL_SIZE))
						{
							return false;
						}

						auto obj_on_counter = std::find_if(
							this->last_obs->objects.cbegin(), this->last_obs->objects.cend(),
							[obj](dhtt_msgs::msg::CookingObject other_obj)
							{
								// Don't match on ourself
								if (other_obj.object_type == obj.object_type)
								{
									return false;
								}
								else
								{
									return obj.location == other_obj.location;
								}
							});
						if (obj_on_counter !=
							this->last_obs->objects.cend()) // if found obj on counter
						{
							return false;
						}
					}
					else if (cond == "Free" and not obj.content_ids.empty())
					{
						return false;
					}
					else if (cond == "Contains" and obj.content_ids.empty())
					{
						return false;
					}
				}
				else if (cond.find("Contains+") != std::string::npos)
				{
					if (CookingBehavior::check_contains_list(cond, obj.content_ids) == false)
					{
						return false;
					}
				}
				else if (std::find(obj.physical_state.begin(), obj.physical_state.end(), cond) ==
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
		RCLCPP_WARN(
			this->pub_node_ptr->get_logger(),
			("Could not find object of type and conds: " + dst_val + " " + dst_conds).c_str());
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

std::string CookingBehavior::which_arm(dhtt::Node *container)
{
	auto owned_resources = container->get_owned_resources(); // segfaults if not a copy
	auto found_gripper = std::find_if(owned_resources.cbegin(), owned_resources.cend(),
									  [](dhtt_msgs::msg::Resource resource)
									  { return resource.type == resource.GRIPPER; });

	if (found_gripper != owned_resources.cend())
	{
		return found_gripper->name;
	}
	else
	{
		return "";
	}
}

bool CookingBehavior::can_work() const
{
	// TODO check good floating point comparisons
	if (this->activation_potential <= FLT_EPSILON)
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "We set activation potential to 0 but the node still ran, skipping work. "
					 "Complain to Tyler.");
		return false;
	}

	if (not this->destination_is_good)
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "Destination is not good but the node still ran. Skipping work.");
		return false;
	}

	return true;
}

std::vector<std::string> CookingBehavior::parse_conds_string(const std::string &conds_str)
{
	std::vector<std::string> conds;
	std::ostringstream cond;

	bool inside_quote = false;
	for (const char c : conds_str)
	{
		if (inside_quote)
		{
			if (c == '"')
			{
				inside_quote = false;
			}
			cond << c;
		}
		else if (c == '"')
		{
			cond << c;
			inside_quote = true;
		}
		else if (std::isspace(c))
		{
			// ignore spaces
		}
		else if (c == ',')
		{
			conds.push_back(cond.str());
			cond = std::ostringstream();
		}
		else
		{
			cond << c;
		}
	}

	// don't forget the last value
	if (not cond.str().empty())
	{
		conds.push_back(cond.str());
	}

	if (inside_quote)
	{
		throw std::runtime_error("Quote not closed in " + conds_str);
	}

	return conds;
}

bool CookingBehavior::check_contains_list(const std::string &cond_string,
										  const std::vector<unsigned long> &obj_contained_ids) const
{
	// lets the user use the pipe '|' to signify alternate contains lists
	// if we want deeper logic, we need a better boolean parser
	auto parse_pipe_delimited = [](const std::string &piped_string)
	{
		std::vector<std::vector<std::string>> nested_conds;
		std::ostringstream cond_string;

		for (const char c : piped_string)
		{
			if (c == '|')
			{
				nested_conds.push_back(CookingBehavior::parse_conds_string(cond_string.str()));
				cond_string = std::ostringstream();
			}
			else
			{
				cond_string << c;
			}
		}
		// don't forget the last value
		const std::string &last_val = cond_string.str();
		if (last_val.empty())
		{
			throw std::runtime_error("Malformed contains list " + piped_string);
		}

		nested_conds.push_back(CookingBehavior::parse_conds_string(last_val));

		return nested_conds;
	};

	// expecting format Contains+"Lettuce, Tomato"
	if (cond_string.find("Contains+\"") != 0)
	{
		throw std::runtime_error("Malformed contains list " + cond_string);
	}

	std::size_t obj_list_starts = cond_string.find("\"") + 1;
	std::size_t obj_list_ends =
		cond_string.find('"', obj_list_starts); // substr is not end-inclusive
	auto contains_lists =
		parse_pipe_delimited(cond_string.substr(obj_list_starts, obj_list_ends - obj_list_starts));

	auto contains_list_match_it = std::find_if(
		contains_lists.cbegin(), contains_lists.cend(),
		[&](const std::vector<std::string> &want_contained_objects)
		{
			if (want_contained_objects.size() == 1 and want_contained_objects[0] == "None")
			{
				return obj_contained_ids.empty();
			}

			if (want_contained_objects.size() != obj_contained_ids.size())
			{
				return false; // don't match on an object that contains more than
							  // what we asked for
			}

			// iterate through object names we want to be contained in obj, this should get to
			// .cend() because all of `want_contained_obj_name` should return true
			// (std::find_if_not)
			auto contained_objs_it = std::find_if_not(
				want_contained_objects.cbegin(), want_contained_objects.cend(),
				[&](const std::string &want_contained_obj_name)
				{
					auto world_id_to_name = [&](const unsigned long &id)
					{
						auto matched_obj_it = std::find_if(
							this->last_obs->objects.cbegin(), this->last_obs->objects.cend(),
							[&](const dhtt_msgs::msg::CookingObject &world_obj)
							{ return world_obj.world_id == id; });
						if (matched_obj_it == this->last_obs->objects.cend())
						{
							throw std::runtime_error("Object in last_obs contains an "
													 "id that does not exist");
						}
						return matched_obj_it->object_type;
					};

					// iterate through the contained object ids, this is the second level of the
					// double for loop, this should not get to .cend() because a world_id should
					// match
					auto contained_objs_id_match_it =
						std::find_if(obj_contained_ids.cbegin(), obj_contained_ids.cend(),
									 [&](const unsigned long &contained_id)
									 {
										 std::string resolved_name = world_id_to_name(contained_id);
										 return resolved_name == want_contained_obj_name;
									 });

					// we want an id to match
					return contained_objs_id_match_it != obj_contained_ids.cend();
				});

			// we want this to reach the end, having checked every object
			return contained_objs_it == want_contained_objects.cend();
		});

	// we want one of the lists to match
	bool to_ret = contains_list_match_it != contains_lists.cend();
	return to_ret;
}

} // namespace dhtt_plugins