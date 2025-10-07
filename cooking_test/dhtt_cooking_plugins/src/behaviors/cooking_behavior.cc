#include "dhtt_cooking_plugins/behaviors/cooking_behavior.hpp"

namespace dhtt_cooking_plugins
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
	// if (static_cast<int>(params.size()) > 5)
	// 	throw std::invalid_argument(
	// 		"Too many parameters passed to node. Coord or object required and "
	// 		"optionally object conditions and mark and unmark.");

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
	this->should_unmark = false;

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
				else if (key == CookingBehavior::PARAM_KEEP)
				{
					this->should_keep = true;
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
	this->updated = true;
}

double CookingBehavior::get_perceived_efficiency(dhtt::Node* container)
{
	// Make sure we're up to date with observations
	// this->com_agg->spin_some();

	(void) container;

	if ( this->done )
		return 0;

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
	else 
	{
		DHTT_LOG_DEBUG(this->com_agg, "Waiting for update");

		auto check_updated = [&](){ return (bool) this->updated; };

		std::unique_lock<std::mutex> update_lock(this->observation_mut);

		if ( not this->updated )
			this->update_condition.wait(update_lock, check_updated);

		DHTT_LOG_DEBUG(this->com_agg, "Update received");

		update_lock.unlock();
	}

	return 1;
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
	geometry_msgs::msg::Point agent_loc = this->observed_agent.object_members.location;
	return this->point_distance(agent_loc, point);
}

void CookingBehavior::initialize(std::vector<std::string> params)
{
	ActionType::initialize(params);

	// subscribers
	this->cooking_observation_subscriber = this->com_agg->register_subscription<dhtt_cooking_msgs::msg::CookingObservation>("Cooking_Observations", "", 
													std::bind(&CookingBehavior::observation_callback, this, std::placeholders::_1));

	// clients
	this->cooking_request_client = this->com_agg->register_service_client<dhtt_cooking_msgs::srv::CookingRequest>("Cooking_Server");

	// checks
	if (not this->cooking_request_client->wait_for_service(std::chrono::seconds(1)))
	{
		DHTT_LOG_FATAL(this->com_agg, "Could not contact dhtt_cooking service");
		throw std::runtime_error("Could not contact dhtt_cooking service");
	}

	// initialize message types to more obvious bad values
	this->destination_point.x = -1;
	this->destination_point.y = -1;
	this->destination_object.world_id = -1;
	this->destination_object.location.x = -1;
	this->destination_object.location.y = -1;

	this->updated = {false};
}

void CookingBehavior::destruct()
{
	this->com_agg->unregister_subscription<dhtt_cooking_msgs::msg::CookingObservation>("Cooking_Observations", this->cooking_observation_subscriber);
}

void CookingBehavior::observation_callback(std::shared_ptr<dhtt_cooking_msgs::msg::CookingObservation> msg)
{
	if (this->done)
	{
		return; // exit early
	}

	{
		std::lock_guard<std::mutex> update_guard(this->observation_mut);
		DHTT_LOG_DEBUG(this->com_agg, "Inside observation callback");
		this->observed_env.clear(); // clear first so we only have the most up to date information
		
		for ( auto iter : msg->objects )
		{
			std::string name = dhtt_cooking_utils::get_resource_name(iter);

			this->observed_env[name] = iter;
		}

		this->observed_agent = msg->agents[0];

		this->updated = true;
	}
	
	this->update_condition.notify_one();
}

std::string CookingBehavior::set_destination_to_closest_free_object(int8_t type, std::vector<dhtt_msgs::msg::Resource> resources_list)
{
	// Assumes on entry that this->last_obs is valid. I.e. we've received an observation and
	// destination_type is object not coord. It is the callers responsibility to be sure of that,
	float min_dist = 10000.0;
	std::string current_min_name = "";

	for ( auto iter : resources_list )
	{
		if ( not iter.locked and iter.type == type and dhtt_cooking_utils::is_free(iter, this->observed_env) )
		{
			float cur_dist = this->agent_point_distance(this->observed_env[iter.name].location);
			if ( cur_dist < min_dist )
			{
				min_dist = cur_dist;
				current_min_name = iter.name;
			}
		}
	}

	this->destination_point = this->observed_env[current_min_name].location;
	this->destination_object = this->observed_env[current_min_name];
	this->destination_is_good = true;

	return current_min_name;
}

std::string CookingBehavior::set_destination_to_closest_owned(int8_t type, std::vector<dhtt_msgs::msg::Resource> resources_list)
{
	float min_dist = 10000.0;
	std::string current_min_name = "";

	for ( auto iter : resources_list )
	{
		int8_t cur_type = iter.type;
		if ( cur_type == type )
		{
			float cur_dist = this->agent_point_distance(this->observed_env[iter.name].location);
			if ( cur_dist < min_dist )
			{
				min_dist = cur_dist;
				current_min_name = iter.name;
			}
		}
	}

	this->destination_point = this->observed_env[current_min_name].location;
	this->destination_object = this->observed_env[current_min_name];
	this->destination_is_good = true;

	return current_min_name;
}

std::string CookingBehavior::set_destination_to_closest(int8_t type, std::vector<dhtt_msgs::msg::Resource> resources_list)
{
	float min_dist = 10000.0;
	std::string current_min_name = "";

	for ( auto iter : resources_list )
	{
		int8_t cur_type = iter.type;
		if ( cur_type == type and not iter.locked )
		{
			float cur_dist = this->agent_point_distance(this->observed_env[iter.name].location);
			if ( cur_dist < min_dist )
			{
				min_dist = cur_dist;
				current_min_name = iter.name;
			}
		}
	}

	this->destination_point = this->observed_env[current_min_name].location;
	this->destination_object = this->observed_env[current_min_name];
	this->destination_is_good = true;

	return current_min_name;
}

std::string CookingBehavior::set_destination_to_closest_free(int8_t type, std::vector<dhtt_msgs::msg::Resource> resources_list)
{
	float min_dist = 10000.0;
	std::string current_min_name = "";

	for ( auto iter : resources_list )
	{
		int8_t cur_type = iter.type;
		if ( cur_type == type and not iter.locked and dhtt_cooking_utils::is_free(iter, this->observed_env) )
		{
			float cur_dist = this->agent_point_distance(this->observed_env[iter.name].location);
			if ( cur_dist < min_dist )
			{
				min_dist = cur_dist;
				current_min_name = iter.name;
			}
		}
	}

	this->destination_point = this->observed_env[current_min_name].location;
	this->destination_object = this->observed_env[current_min_name];
	this->destination_is_good = true;

	return current_min_name;
}

std::string CookingBehavior::set_destination_to_static_under(int8_t type, std::vector<dhtt_msgs::msg::Resource> resources_list)
{
	// first find resource in given list
	dhtt_msgs::msg::Resource dynamic_resource;

	auto find_by_type = [&type](dhtt_msgs::msg::Resource to_check){ return to_check.type == type; };

	// should always be just one of each type owned by the move behavior
	auto dynamic_resource_iter = std::find_if(resources_list.begin(), resources_list.end(), find_by_type);

	if ( dynamic_resource_iter == resources_list.end() )
		return "";

	dynamic_resource = *dynamic_resource_iter;
	
	// next do linear search to find the static resource underneath
	auto same_location = []( geometry_msgs::msg::Point loc1, geometry_msgs::msg::Point loc2 )  { return loc1.x == loc2.x and loc1.y == loc2.y; };

	auto dynamic_loc = this->observed_env[dynamic_resource.name].location;

	auto find_by_location = [&dynamic_loc, &same_location](std::pair<std::string, dhtt_cooking_msgs::msg::CookingObject> to_check) 
	{ 
		return same_location(dynamic_loc, to_check.second.location) and dhtt_cooking_utils::is_static_obj(to_check.second); 
	};

	auto static_object = std::find_if(this->observed_env.begin(), this->observed_env.end(), find_by_location);

	this->destination_point = static_object->second.location;
	this->destination_object = static_object->second;
	this->destination_is_good = true;

	return static_object->first;
}

std::string CookingBehavior::get_dynamic_in_front()
{
	// get location and orientation of agent
	auto agent_loc = this->observed_agent.object_members.location;
	auto agent_ori = this->observed_agent.orientation;

	// get location of in front
	auto front_loc = agent_loc;

	if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::WEST )
		front_loc.x -= 1;
	else if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::EAST )
		front_loc.x += 1;
	else if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::SOUTH )
		front_loc.y += 1;
	else if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::NORTH )
		front_loc.y -= 1;

	// get dynamic objects in front
	std::map<std::string, dhtt_cooking_msgs::msg::CookingObject>::iterator found_iter = this->observed_env.begin();

	auto find_front_loc = [&front_loc]( std::pair<std::string, dhtt_cooking_msgs::msg::CookingObject> to_check ){ return to_check.second.location.x == front_loc.x and to_check.second.location.y == front_loc.y; };

	// continue the search until the object found is dynamic or we run out
	do
	{
		found_iter = std::find_if(std::next(found_iter), this->observed_env.end(), find_front_loc);

		if ( found_iter == this->observed_env.end() )
			return "";

	} while ( not dhtt_cooking_utils::is_dynamic_obj( found_iter->second ) );

	// pick one and return
	return found_iter->first;
}

std::string CookingBehavior::get_static_in_front()
{
	// get location and orientation of agent
	auto agent_loc = this->observed_agent.object_members.location;
	auto agent_ori = this->observed_agent.orientation;

	// get location of in front
	auto front_loc = agent_loc;

	if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::WEST )
		front_loc.x -= 1;
	else if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::EAST )
		front_loc.x += 1;
	else if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::SOUTH )
		front_loc.y += 1;
	else if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::NORTH )
		front_loc.y -= 1;

	// get dynamic objects in front
	auto find_front_loc = [&front_loc]( std::pair<std::string, dhtt_cooking_msgs::msg::CookingObject> to_check ){ return to_check.second.location.x == front_loc.x and to_check.second.location.y == front_loc.y; };
	std::map<std::string, dhtt_cooking_msgs::msg::CookingObject>::iterator found_iter = std::find_if(this->observed_env.begin(), this->observed_env.end(), find_front_loc);

	if ( found_iter == this->observed_env.end() )
		return "";

	// continue the search until the object found is dynamic or we run out
	while ( not dhtt_cooking_utils::is_static_obj( found_iter->second ) )
	{
		found_iter = std::find_if(std::next(found_iter), this->observed_env.end(), find_front_loc);

		if ( found_iter == this->observed_env.end() )
			return "";

	}

	// pick one and return
	return found_iter->first;
}

std::vector<std::string> CookingBehavior::get_all_dynamic_in_front()
{
	// get location and orientation of agent
	auto agent_loc = this->observed_agent.object_members.location;
	auto agent_ori = this->observed_agent.orientation;

	// get location of in front
	auto front_loc = agent_loc;

	if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::WEST )
		front_loc.x -= 1;
	else if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::EAST )
		front_loc.x += 1;
	else if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::SOUTH )
		front_loc.y += 1;
	else if ( agent_ori == dhtt_cooking_msgs::msg::CookingAgent::NORTH )
		front_loc.y -= 1;

	std::vector<std::string> to_ret;

	auto find_front_loc = [&front_loc]( std::pair<std::string, dhtt_cooking_msgs::msg::CookingObject> to_check ){ return to_check.second.location.x == front_loc.x and to_check.second.location.y == front_loc.y; };

	std::map<std::string, dhtt_cooking_msgs::msg::CookingObject>::iterator found_iter = std::find_if(this->observed_env.begin(), this->observed_env.end(), find_front_loc);
	// continue the search until all dynamic objs are found
	do 
	{
		if ( dhtt_cooking_utils::is_dynamic_obj( found_iter->second ) )
			to_ret.push_back(found_iter->first);
	} while ( ( found_iter = std::find_if(std::next(found_iter), this->observed_env.end(), find_front_loc) ) != this->observed_env.end() ) ;

	// pick one and return
	return to_ret;
}

std::string CookingBehavior::get_static_underneath(std::string name)
{ 
	auto dynamic_obj = this->observed_env[name];
	auto same_location = []( geometry_msgs::msg::Point loc1, geometry_msgs::msg::Point loc2 )  { return loc1.x == loc2.x and loc1.y == loc2.y; };

	for ( auto pair : this->observed_env )
		if ( dhtt_cooking_utils::is_static_obj(pair.second) and same_location(dynamic_obj.location, pair.second.location) )
			return pair.first;

	return "";
}

std::string CookingBehavior::has_resource_free_of_type(dhtt::Node* container, int8_t type)
{
	auto find_by_type = [&type](dhtt_msgs::msg::Resource to_check){ return type == to_check.type and not to_check.locked; };
	auto resources = container->get_resource_state();
	
	auto iter = std::find_if(resources.begin(), resources.end(), find_by_type);

	if ( iter != resources.end() )
		return iter->name;

	return ""; 
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
		DHTT_LOG_ERROR(this->com_agg,
					 "Destination is not good but the node still ran. Skipping work.");
		return false;
	}

	return true;
}

bool CookingBehavior::mark_object(unsigned long object_id, const std::string &mark) const
{
	// if we don't have a mark then this automatically should succeed
	if ( not strcmp(mark.c_str(), "") )
		return true;

	auto object_exists_it =
		std::find_if(this->last_obs->objects.cbegin(), this->last_obs->objects.cend(),
					 [object_id](const dhtt_cooking_msgs::msg::CookingObject &obj)
					 { return obj.world_id == object_id; });

	if (object_exists_it == this->last_obs->objects.cend())
	{
		DHTT_LOG_ERROR(this->com_agg,
					 "Tried to mark object " << object_id << " but it doesn't exist");
		return false;
	}

	auto existing_marked_objects = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS).as_integer_array();
	auto existing_marked_objects_taints = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS_TAINTS).as_string_array();
	auto existing_marked_objects_types = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS_TYPES).as_string_array();

	if (existing_marked_objects.size() != existing_marked_objects_taints.size())
	{
		DHTT_LOG_FATAL(this->com_agg,
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

		DHTT_LOG_ERROR(this->com_agg,
					 "Tried to mark object " << object_id << " but it is already marked");
		return false;
	}

	existing_marked_objects.push_back(object_id);
	existing_marked_objects_taints.push_back(mark);
	existing_marked_objects_types.push_back(this->destination_object.object_type);

	this->com_agg->set_parameters_sync(
		{rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS, existing_marked_objects),
		 rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS_TAINTS,
						   existing_marked_objects_taints),
		 rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS_TYPES,
						   existing_marked_objects_types)});

	return true;
}

bool CookingBehavior::unmark_static_object_under_obj(const dhtt_cooking_msgs::msg::CookingObject &obj, bool force) const
{
	for (const auto &world_obj : this->last_obs->objects)
	{
		if (world_obj.is_static and world_obj.location == obj.location and not this->unmark_object(world_obj.world_id, force) )
		{
			// should only be one static object at a location
			return false;
		}
	}

	return true;
}

bool CookingBehavior::unmark_at_location(const geometry_msgs::msg::Point& loc)
{
	for ( const auto &world_obj : this->last_obs->objects )
	{
		if ( world_obj.location == loc and not this->unmark_object(world_obj.world_id, true) )
		{
			return false;
		}
	}

	return true;
}

void CookingBehavior::unmark_all_nonexistant()
{
	auto marked_object_ids = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS).as_integer_array();

	long int to_find;
	auto match_id = [&to_find](const dhtt_cooking_msgs::msg::CookingObject to_check)
	{
		return to_find == (int) to_check.world_id;
	};

	std::vector<long int> not_found;

	for ( const auto id : marked_object_ids )
	{
		auto iter = std::find_if(this->last_obs->objects.begin(), this->last_obs->objects.end(), match_id);

		if ( iter != this->last_obs->objects.end() )
			not_found.push_back(id);

	}

	for ( long int iter : not_found )
		this->unmark_object(iter, true);
}

bool CookingBehavior::unmark_object(unsigned long object_id, bool force) const
{

	auto marked_object_ids = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS).as_integer_array();
	auto marked_object_taints = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS_TAINTS).as_string_array();
	auto marked_objects_types = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS_TYPES).as_string_array();

	auto found_marked_id_it = std::find_if(marked_object_ids.cbegin(), marked_object_ids.cend(),
										   [object_id](const int64_t &other_obj_id)
										   { return other_obj_id == (int) object_id; });

	if (found_marked_id_it == marked_object_ids.cend())
	{
		DHTT_LOG_WARN(this->com_agg,
					"Tried to unmark object " << object_id << " but it is not marked");
		return false;
	}

	const auto found_index = std::distance(marked_object_ids.cbegin(), found_marked_id_it);
	const auto &found_taint = marked_object_taints[found_index];
	
	if (not force and this->destination_mark != found_taint)
	{
		DHTT_LOG_ERROR(this->com_agg, 
					 "Tried to unmark object " << object_id << " but it is not marked with our taint");
		return false;
	}

	marked_object_ids.erase(found_marked_id_it);
	marked_object_taints.erase(marked_object_taints.begin() + found_index);
	marked_objects_types.erase(marked_objects_types.begin() + found_index);

	this->com_agg->set_parameters_sync(
		{rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS, marked_object_ids),
		 rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS_TAINTS, marked_object_taints),
		 rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS_TYPES, marked_objects_types)});

	return true;
}

bool CookingBehavior::unmark_given_type(const geometry_msgs::msg::Point& loc, std::string type)
{
	for ( const auto &world_obj : this->last_obs->objects )
	{
		if ( world_obj.location == loc and world_obj.object_type == type and not this->unmark_object(world_obj.world_id, true) )
		{
			return false;
		}
	}

	return true;
}

void CookingBehavior::clean_marks()
{
	if ( not strcmp(this->destination_mark.c_str(), "") )
		return;

	// first get the parameters in question
	auto marked_object_ids = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS).as_integer_array();
	auto marked_object_taints = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS_TAINTS).as_string_array();
	auto marked_objects_types = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS_TYPES).as_string_array();

	int to_find;
	auto find_obj_by_id = [&to_find](dhtt_cooking_msgs::msg::CookingObject to_check){ return to_check.world_id == to_find; };

	std::vector<int> indices_to_remove;

	for ( int i = 0; i < (int) marked_object_ids.size() ; i++ )
	{
		to_find = marked_object_ids[i];

		// if the object has our mark and doesn't exist in the last_obs
		if ( marked_object_taints[i] == this->destination_mark and 
				std::find_if(this->last_obs->objects.begin(), this->last_obs->objects.end(), find_obj_by_id) == this->last_obs->objects.end() )
			indices_to_remove.push_back(i);
	}

	for ( auto index_iter = indices_to_remove.rbegin() ; index_iter != indices_to_remove.rend() ; index_iter++ )
	{
		marked_object_ids.erase(marked_object_ids.begin() + *index_iter);	
		marked_object_taints.erase(marked_object_taints.begin() + *index_iter);	
		marked_objects_types.erase(marked_objects_types.begin() + *index_iter);	
	}

	this->com_agg->set_parameters_sync(
		{rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS, marked_object_ids),
		 rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS_TAINTS, marked_object_taints),
		 rclcpp::Parameter(CookingBehavior::PARAM_MARK_OBJECTS_TYPES, marked_objects_types)});
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
							[&](const dhtt_cooking_msgs::msg::CookingObject &world_obj)
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
								  const dhtt_cooking_msgs::msg::CookingObject &obj) const
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
								 [obj](const dhtt_cooking_msgs::msg::CookingObject &other_obj)
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

char CookingBehavior::check_mark(const dhtt_cooking_msgs::msg::CookingObject &obj) const
{
	const std::vector<long int> marked_object_ids = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS).as_integer_array();

	const std::vector<std::string> marked_object_taints = this->com_agg->get_parameter_sync(CookingBehavior::PARAM_MARK_OBJECTS_TAINTS).as_string_array();

	// three cases: 0. object is marked but with another taint; 1. object is marked with our
	// taint; 2. object not marked at all;
	auto found_marked_id_it =
		std::find_if(marked_object_ids.cbegin(), marked_object_ids.cend(),
					 [obj](int64_t other_obj_id) { return other_obj_id == (int) obj.world_id; });

	if (found_marked_id_it == marked_object_ids.cend())
	{
		return '2'; // not marked at all
	}

	const auto found_index = std::distance(marked_object_ids.cbegin(), found_marked_id_it);
	const auto &found_taint = marked_object_taints[found_index];
	if (this->destination_mark != found_taint and found_taint != CookingBehavior::SPECIAL_MARK)
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

std::shared_future<dhtt_cooking_msgs::srv::CookingRequest::Response::SharedPtr> CookingBehavior::send_request_and_update(dhtt_cooking_msgs::srv::CookingRequest::Request::SharedPtr to_send)
{
	{
		using namespace std::chrono_literals;

		if ( not this->cooking_request_client->wait_for_service(1s) )
			throw std::runtime_error("Lost connection to the service!");
	}

	this->updated = false;// this timing is probably still not guaranteed to work but fingers crossed
	auto res = this->cooking_request_client->async_send_request(to_send);

	// first wait for the response
	res.wait();	

	// now wait for the udpate from the server
	auto check_updated = [&](){ return true; }; // we always wake up on notifications

	std::unique_lock<std::mutex> update_lock(this->observation_mut);

	if ( not this->updated )
	{
		this->update_condition.wait(update_lock, check_updated);

		update_lock.unlock(); 
	}

	return res;
}

} // namespace dhtt_plugins