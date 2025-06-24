#include "dhtt_plugins/behaviors/cooking_behavior.hpp"

namespace dhtt_plugins
{

void CookingBehavior::parse_params(std::vector<std::string> params)
{
	std::pair<std::string, std::string> kv;
	std::string key, value;
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
	if (static_cast<int>(params.size()) > 4)
		throw std::invalid_argument(
			"Too many parameters passed to node. Coord or object required and "
			"optionally object conditions and mark and unmark.");

	if (static_cast<int>(params.size()) == 0)
	{
		throw std::invalid_argument(
			"Not enough parameters passed to node. Coord or Object required and "
			"optionally object conditions and mark.");
	}

	/* Coord/Object type */
	kv = CookingBehavior::extract_keyval(params[0]);
	key = kv.first;
	value = kv.second;

	if (not(key == CookingBehavior::PARAM_COORDINATE or key == CookingBehavior::PARAM_OBJECT_TYPE))
		throw std::invalid_argument(
			"Expected parameter " + std::string(CookingBehavior::PARAM_COORDINATE) + " or " +
			CookingBehavior::PARAM_OBJECT_TYPE + ", but received " + key + ". Returning in error.");
	this->destination_type = key;
	this->destination_value = value;

	if (this->destination_type == CookingBehavior::PARAM_COORDINATE)
	{
		this->destination_point = parse_coord_string(this->destination_value);
		this->destination_is_good = true;
	}
	else if (this->destination_type == CookingBehavior::PARAM_OBJECT_TYPE)
	{
		if (params.size() < 2) // only specified object type, no conditions or mark
		{
			this->destination_conditions = "";
			this->destination_mark = "";
		}
		else
		{
			/* Object Conditions and Marks */
			for (auto param = params.cbegin() + 1; param < params.cend(); ++param)
			{
				kv = CookingBehavior::extract_keyval(*param);
				key = kv.first;
				value = kv.second;

				if (key == CookingBehavior::PARAM_OBJECT_CONDITIONS)
				{
					this->destination_conditions = value;
				}
				else if (key == CookingBehavior::PARAM_OBJECT_MARK)
				{
					this->destination_mark = value;
				}
				else if (key == CookingBehavior::PARAM_UNMARK)
				{
					this->should_unmark = true;
				}
				else
				{
					throw std::invalid_argument(
						"Expected parameter " +
						std::string(CookingBehavior::PARAM_OBJECT_CONDITIONS) + " or " +
						std::string(CookingBehavior::PARAM_MARK_OBJECTS) + "or" +
						std::string(CookingBehavior::PARAM_UNMARK) + ", but received " + key);
				}
			}
		}
	}
	else
	{
		throw std::invalid_argument(
			"Expected parameter " + std::string(CookingBehavior::PARAM_OBJECT_TYPE) + " or " +
			std::string(CookingBehavior::PARAM_COORDINATE) + ", but received " + key);
	}

	this->params = params;
}

double CookingBehavior::get_perceived_efficiency()
{
	// Make sure we're up to date with observations
	this->executor->spin_some(std::chrono::nanoseconds(0));

	if (not this->last_obs)
	{
		RCLCPP_WARN(this->pub_node_ptr->get_logger(),
					"No observation yet. This node probably won't behave as intended. Making "
					"Observe request");

		// Make sure we get an initial observation
		auto req = std::make_shared<dhtt_msgs::srv::CookingRequest::Request>();
		req->super_action = dhtt_msgs::srv::CookingRequest::Request::OBSERVE;
		auto res = this->cooking_request_client->async_send_request(req);
		RCLCPP_DEBUG(this->pub_node_ptr->get_logger(), "Sent initial observation request");
		this->executor->spin_until_future_complete(res, std::chrono::seconds(1));

		if (res.valid())
		{
			RCLCPP_DEBUG(this->pub_node_ptr->get_logger(),
						 "Received initial observation request: We're good to go.");
		}
		else
		{
			RCLCPP_ERROR(this->pub_node_ptr->get_logger(), "BUG, observation request timed out");
		}

		while (not this->last_obs)
			;

		// no return
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

void CookingBehavior::initialize(std::vector<std::string> params)
{
	ActionType::initialize(params);

	// subscribers
	// this->cooking_observation_subscriber_name = this->com_agg->register_subscription<dhtt_msgs::msg::CookingObservation>("Cooking_Observations", "", 
	// 												std::bind(&CookingBehavior::observation_callback, this, std::placeholders::_1));

	this->cooking_observation_subscriber =
		this->pub_node_ptr->create_subscription<dhtt_msgs::msg::CookingObservation>(
			"Cooking_Observations", 10,
			std::bind(&CookingBehavior::observation_callback, this, std::placeholders::_1));

	// clients
	// this->com_agg->register_service_client<dhtt_msgs::srv::CookingRequest>("Cooking_Server");

	this->cooking_request_client =
		this->pub_node_ptr->create_client<dhtt_msgs::srv::CookingRequest>("Cooking_Server");

	this->param_node_ptr =
		std::make_shared<rclcpp::Node>(std::string(this->pub_node_ptr->get_name()) + "_params",
									   rclcpp::NodeOptions()
										   .allow_undeclared_parameters(true)
										   .automatically_declare_parameters_from_overrides(true));
	this->params_client_ptr =
		std::make_shared<rclcpp::SyncParametersClient>(this->param_node_ptr, "/param_node");

	// checks
	if (not this->cooking_request_client->wait_for_service(std::chrono::seconds(1)))
	{
		RCLCPP_FATAL(this->pub_node_ptr->get_logger(), "Could not contact dhtt_cooking service");
		throw std::runtime_error("Could not contact dhtt_cooking service");
	}

	if (not this->params_client_ptr->wait_for_service(std::chrono::seconds(1)))
	{
		RCLCPP_FATAL(this->pub_node_ptr->get_logger(), "Could not contact param node");
		throw std::runtime_error("Could not contact param node");
	}

	if (not this->last_obs)
	{
		RCLCPP_FATAL(this->pub_node_ptr->get_logger(),
					 "Did not get first observation. This node may not behave properly.");
	}

	// initialize message types to more obvious bad values
	this->destination_point.x = -1;
	this->destination_point.y = -1;
	this->destination_object.world_id = -1;
	this->destination_object.location.x = -1;
	this->destination_object.location.y = -1;
}

void CookingBehavior::observation_callback(std::shared_ptr<dhtt_msgs::msg::CookingObservation> msg)
{
	if (this->done)
	{
		return; // exit early
	}

	RCLCPP_DEBUG(this->pub_node_ptr->get_logger(), "Inside observation callback");
	this->last_obs = msg;

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
	if (this->last_obs == nullptr or this->destination_type == CookingBehavior::PARAM_COORDINATE)
	{
		throw std::runtime_error("Bad call to set_destination_to_closest_object()");
	}

	if (this->destination_value.empty())
	{
		throw std::runtime_error("Empty destination_value");
	}

	RCLCPP_DEBUG(this->pub_node_ptr->get_logger(), "Setting destination to val: %s, with conds: %s",
				 this->destination_value.c_str(), this->destination_conditions.c_str());

	auto pred_typeandconds = [this](const dhtt_msgs::msg::CookingObject &obj)
	{
		if (obj.location == this->last_obs->agents.front().object_members.location)
		{
			return false; // Don't count objects the agent is already holding
		}

		if (obj.object_type == this->destination_value)
		{
			// Check conditions
			const std::vector<std::string> conds =
				CookingBehavior::parse_conds_string(this->destination_conditions);
			if (this->check_conds(conds, obj) == false)
			{
				return false;
			}

			return true; // Correct type and conditions
		}
		return false; // Not correct type
	};

	std::vector<std::vector<dhtt_msgs::msg::CookingObject>::const_iterator> found_typeconds_iters;
	std::vector<std::vector<dhtt_msgs::msg::CookingObject>::const_iterator>
		found_typecondsmark_iters;
	for (auto iter = this->last_obs->objects.cbegin(); iter != this->last_obs->objects.cend();
		 ++iter)
	{
		if (pred_typeandconds(*iter))
		{
			found_typeconds_iters.push_back(iter);
		}
	}

	if (found_typeconds_iters.empty())
	{
		RCLCPP_WARN(this->pub_node_ptr->get_logger(),
					("Could not find object of type and conds: " + this->destination_value + " " +
					 this->destination_conditions)
						.c_str());
		this->destination_is_good = false;
		return;
	}

	// We have some objects in the world that match our type and conditions. Now check if any of
	// them have our mark. If we can't find one because they are all marked by someone else, we
	// fail. If because at least one of them is not marked at all, return that one.
	if (not this->destination_mark.empty())
	{
		std::vector<std::vector<dhtt_msgs::msg::CookingObject>::const_iterator> found_with_no_mark;

		for (auto iter : found_typeconds_iters)
		{
			char mark = this->check_mark(*iter);

			if (mark == '1')
			{
				found_typecondsmark_iters.push_back(iter);
			}
			else if (mark == '2')
			{
				found_with_no_mark.push_back(iter);
			}
			else
			{
				continue;
			}
		}
		if (found_typecondsmark_iters
				.empty()) // none of our taints found, reconsider with unmarked objects
		{
			found_typecondsmark_iters = found_with_no_mark;
		}

		if (found_typecondsmark_iters.empty())
		{
			RCLCPP_WARN(this->pub_node_ptr->get_logger(),
						("Found object of correct type and cond, but incorrect mark: " +
						 this->destination_mark)
							.c_str());
			this->destination_is_good = false;
			return;
		}
	}

	auto found_iters = this->destination_mark.empty() ? std::move(found_typeconds_iters)
													  : std::move(found_typecondsmark_iters);

	const auto found_min_it =
		std::min_element(found_iters.cbegin(), found_iters.cend(),
						 [this](std::vector<dhtt_msgs::msg::CookingObject>::const_iterator itL,
								std::vector<dhtt_msgs::msg::CookingObject>::const_iterator itR)
						 {
							 return this->agent_point_distance(itL->location) <
									this->agent_point_distance(itR->location);
						 });

	this->destination_point = (*found_min_it)->location;
	this->destination_object = **found_min_it;
	this->destination_is_good = true;
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
	if (not this->destination_is_good)
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "Destination is not good but the node still ran. Skipping work.");
		return false;
	}

	return true;
}

bool CookingBehavior::mark_object(unsigned long object_id, const std::string &mark) const
{
	auto object_exists_it =
		std::find_if(this->last_obs->objects.cbegin(), this->last_obs->objects.cend(),
					 [object_id](const dhtt_msgs::msg::CookingObject &obj)
					 { return obj.world_id == object_id; });

	if (object_exists_it == this->last_obs->objects.cend())
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "Tried to mark object %lu but it doesn't exist", object_id);
		return false;
	}

	auto existing_marked_objects = this->params_client_ptr->get_parameter<std::vector<long int>>(
		CookingBehavior::PARAM_MARK_OBJECTS, {});
	auto existing_marked_objects_taints =
		this->params_client_ptr->get_parameter<std::vector<std::string>>(
			CookingBehavior::PARAM_MARK_OBJECTS_TAINTS, {});
	auto existing_marked_objects_types =
		this->params_client_ptr->get_parameter<std::vector<std::string>>(
			CookingBehavior::PARAM_MARK_OBJECTS_TYPES, {});

	if (existing_marked_objects.size() != existing_marked_objects_taints.size())
	{
		RCLCPP_FATAL(this->pub_node_ptr->get_logger(),
					 "Marked object IDs and their Taints do not match up");
		return false;
	}

	if (std::find(existing_marked_objects.cbegin(), existing_marked_objects.cend(), object_id) !=
		existing_marked_objects.cend())
	{
		int index = std::distance(
			existing_marked_objects.cbegin(),
			std::find(existing_marked_objects.cbegin(), existing_marked_objects.cend(), object_id));

		// if the object already has our mark that's fine too
		if (existing_marked_objects_taints[index] == mark)
			return true;

		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "Tried to mark object %lu but it is already marked", object_id);
		return false;
	}

	existing_marked_objects.push_back(object_id);
	existing_marked_objects_taints.push_back(mark);
	existing_marked_objects_types.push_back(this->destination_object.object_type);

	this->params_client_ptr->set_parameters(
		{rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS, existing_marked_objects),
		 rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS_TAINTS,
						   existing_marked_objects_taints),
		 rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS_TYPES,
						   existing_marked_objects_types)});

	return true;
}

bool CookingBehavior::unmark_static_object_under_obj(const dhtt_msgs::msg::CookingObject &obj) const
{
	for (const auto &world_obj : this->last_obs->objects)
	{
		if (world_obj.is_static and world_obj.location == obj.location)
		{
			// should only be one static object at a location
			return this->unmark_object(world_obj.world_id);
		}
	}

	RCLCPP_ERROR(
		this->pub_node_ptr->get_logger(),
		("Tried to unmark static object but none under object " + obj.object_type).c_str());
	return false;
}

bool CookingBehavior::unmark_object(unsigned long object_id) const
{
	std::vector<long int> marked_object_ids =
		this->params_client_ptr->get_parameter<std::vector<long int>>(
			CookingBehavior::PARAM_MARK_OBJECTS, {});

	std::vector<std::string> marked_object_taints =
		this->params_client_ptr->get_parameter<std::vector<std::string>>(
			CookingBehavior::PARAM_MARK_OBJECTS_TAINTS, {});

	auto marked_objects_types = this->params_client_ptr->get_parameter<std::vector<std::string>>(
		CookingBehavior::PARAM_MARK_OBJECTS_TYPES, {});

	auto found_marked_id_it = std::find_if(marked_object_ids.cbegin(), marked_object_ids.cend(),
										   [object_id](const int64_t &other_obj_id)
										   { return other_obj_id == object_id; });

	if (found_marked_id_it == marked_object_ids.cend())
	{
		RCLCPP_WARN(this->pub_node_ptr->get_logger(),
					"Tried to unmark object %lu but it is not marked", object_id);
		return false;
	}

	const auto found_index = std::distance(marked_object_ids.cbegin(), found_marked_id_it);
	const auto &found_taint = marked_object_taints[found_index];
	if (this->destination_mark != found_taint)
	{
		RCLCPP_ERROR(this->pub_node_ptr->get_logger(),
					 "Tried to unmark object %lu but it is not marked with our taint", object_id);
		return false;
	}

	marked_object_ids.erase(found_marked_id_it);
	marked_object_taints.erase(marked_object_taints.begin() + found_index);
	marked_objects_types.erase(marked_objects_types.begin() + found_index);

	this->params_client_ptr->set_parameters(
		{rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS, marked_object_ids),
		 rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS_TAINTS, marked_object_taints),
		 rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS_TYPES, marked_objects_types)});

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

bool CookingBehavior::check_conds(const std::vector<std::string> &conds,
								  const dhtt_msgs::msg::CookingObject &obj) const
{
	// Note that if conds is empty (user provided empty string ""), we select the closest of
	// any condition.
	for (const auto &cond : conds)
	{
		if (cond == "Free" or cond == "Contains")
		{
			// Counters are not ContentObjects
			if (cond == "Free" and obj.object_type == "Counter")
			{
				// Check counter isn't an unreachable corner
				if ((obj.location.x == 0 and obj.location.y == 0) or
					(obj.location.x == 0 and obj.location.y == CookingBehavior::LEVEL_SIZE) or
					(obj.location.x == CookingBehavior::LEVEL_SIZE and obj.location.y == 0) or
					(obj.location.x == CookingBehavior::LEVEL_SIZE and
					 obj.location.y == CookingBehavior::LEVEL_SIZE))
				{
					return false;
				}

				auto obj_on_counter =
					std::find_if(this->last_obs->objects.cbegin(), this->last_obs->objects.cend(),
								 [obj](const dhtt_msgs::msg::CookingObject &other_obj)
								 {
									 // Don't match on ourself and cannot have two Counters in the
									 // same place
									 if (other_obj.object_type == obj.object_type)
									 {
										 return false;
									 }
									 else
									 {
										 return obj.location == other_obj.location;
									 }
								 });
				if (obj_on_counter != this->last_obs->objects.cend()) // if found obj on counter
				{
					return false;
				}
			}
			else if ((cond == "Free" and not obj.content_ids.empty()) or
					 (cond == "Contains" and obj.content_ids.empty()))
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

	return true; // all conditions satisfied
}

char CookingBehavior::check_mark(const dhtt_msgs::msg::CookingObject &obj) const
{
	const std::vector<long int> marked_object_ids =
		this->params_client_ptr->get_parameter<std::vector<long int>>(
			CookingBehavior::PARAM_MARK_OBJECTS, {});

	const std::vector<std::string> marked_object_taints =
		this->params_client_ptr->get_parameter<std::vector<std::string>>(
			CookingBehavior::PARAM_MARK_OBJECTS_TAINTS, {});

	// three cases: 0. object is marked but with another taint; 1. object is marked with our
	// taint; 2. object not marked at all;
	auto found_marked_id_it =
		std::find_if(marked_object_ids.cbegin(), marked_object_ids.cend(),
					 [obj](int64_t other_obj_id) { return other_obj_id == obj.world_id; });

	if (found_marked_id_it == marked_object_ids.cend())
	{
		return '2'; // not marked at all
	}

	const auto found_index = std::distance(marked_object_ids.cbegin(), found_marked_id_it);
	const auto &found_taint = marked_object_taints[found_index];
	if (this->destination_mark != found_taint)
	{
		return '0'; // object is marked with another taint
	}

	return '1'; // object is marked with our taint
}

std::pair<std::string, std::string> CookingBehavior::extract_keyval(const std::string &param_string)
{
	auto separator_pos = param_string.find(": ");
	if (separator_pos == std::string::npos)
		throw std::invalid_argument(
			"Parameters are expected in the format \"key: value\" but received in the form " +
			param_string + ". Returning in error.");

	std::string key = param_string.substr(0, separator_pos);
	std::string value = param_string.substr(separator_pos + 2, param_string.size() - separator_pos);
	return std::make_pair(key, value);
}

} // namespace dhtt_plugins