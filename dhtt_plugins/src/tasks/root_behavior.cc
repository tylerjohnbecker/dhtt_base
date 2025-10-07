#include "dhtt_plugins/tasks/root_behavior.hpp"

namespace dhtt_plugins
{
	void RootBehavior::initialize(std::vector<std::string> params)
	{
		// all these will not be performed after resetting the tree
		if ( not this->already_made )
		{
			std::string resources_topic = std::string(TREE_PREFIX) + RESOURCES_POSTFIX;
			std::string control_topic = std::string(TREE_PREFIX) + CONTROL_POSTFIX;	
	
			this->status_pub = this->com_agg->register_publisher<dhtt_msgs::msg::Resources>(resources_topic);
			this->control_server = this->com_agg->create_service<dhtt_msgs::srv::InternalControlRequest>(control_topic, std::bind(&RootBehavior::control_callback, this, std::placeholders::_1, std::placeholders::_2));
		
			this->parse_params(params);
		}

		this->children_done = false;
		this->already_made = true;

		this->children_allowed = true;
		this->interrupted = false;

		this->params = params;
		this->slow = (int) params.size() == 2;
		this->load_resources_from_yaml();

		this->resource_update_mut_ptr = this->com_agg->request_mutex("resource_update");
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> RootBehavior::auction_callback( dhtt::Node* container ) 
	{
		auto print_resource_state = [&]()
		{
			DHTT_LOG_FATAL(this->com_agg, "\tCurrent state of resources:");

			for ( auto iter : this->canonical_resources_list )
			{
				DHTT_LOG_FATAL(this->com_agg, "\t\t" << iter.name << " - type: " << (int) iter.type << ", locked: " << iter.locked 
					<< ", owners: " << (int) iter.owners << ", channel: " << (int) iter.channel);
			}
		};

		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		DHTT_LOG_INFO(this->com_agg, "\n\n\n\tActivation received starting task execution...");

		this->children_done = false;
		this->interrupted = false;

		// print_resource_state();

		// this relies on there only being one direct child of the root node (should enforce this as well)
		while ( not this->is_done() and not this->interrupted )
		{
			{ // scope in which the resource state can be updated
				std::lock_guard<std::mutex> resource_update_lock(*(this->resource_update_mut_ptr));

				// publish state of resources to all nodes in the tree
				// this->publish_resources();
				this->pull_resources();
				this->push_resources(); 

				// activate children
				dhtt_msgs::action::Activation::Goal n_goal;

				DHTT_LOG_FATAL(this->com_agg, "\tSpreading activation from root...");

				container->activate_all_children(n_goal);

				DHTT_LOG_FATAL(this->com_agg, "\tWaiting for response...\n");

				// get responses
				container->block_for_activation_from_children();

				auto result = container->get_activation_results();

				if ( result.empty() )
					DHTT_LOG_ERROR(this->com_agg, "Empty response received!!!");
				
				if ( (*result.begin()).second.done )
				{
					DHTT_LOG_FATAL(this->com_agg, "\tChildren done early!");

					this->children_done = (*result.begin()).second.done;
					break;
				}

				if ( not (*result.begin()).second.possible )
				{
					DHTT_LOG_FATAL(this->com_agg, "Children not possible, trying again...");
					container->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

					to_ret->done = false;

					return to_ret;

					// continue;
				}
				// print_resource_state();

				// update resources
				n_goal.granted_resources = this->give_resources( (*result.begin()).second.requested_resources );
				n_goal.success = true; 

				DHTT_LOG_FATAL(this->com_agg, "\tRequest accepted!\n");

				// activate children
				container->activate_all_children(n_goal);
				// get responses
				container->block_for_activation_from_children();

				result = container->get_activation_results();

				// update resources (release the ones in passed resources because if they make it to root then they have to be released anyway)
				DHTT_LOG_FATAL(this->com_agg, "\tRequest complete, updating resources!\n\n --- --- ---");

				// add and remove dynamic resources from the result ( resources are added locked and then released if in released or passed resource lists )
				this->add_resources( (*result.begin()).second.added_resources );
				this->remove_resources( (*result.begin()).second.removed_resources );

				// behaviors are allowed to request resources after execution as well
				n_goal.granted_resources = this->give_resources( (*result.begin()).second.requested_resources );

				this->release_resources( (*result.begin()).second.released_resources );
				this->release_resources( (*result.begin()).second.passed_resources );
				
				this->children_done = (*result.begin()).second.done;

				this->push_resources();
			}

			print_resource_state();

			if (this->slow)
				rclcpp::sleep_for(std::chrono::milliseconds(150));
		}
		
		this->release_all_resources(); // might be improper for stopping and starting the tree

		// ensure that resources are released before next activation
		if ( this->is_done() )
		{
			DHTT_LOG_INFO(this->com_agg, "All children done. Task successfully completed!\n\n\n");
			container->update_status(dhtt_msgs::msg::NodeStatus::DONE);
		}
		else
		{
			DHTT_LOG_INFO(this->com_agg, "Task interrupted. Returning to state WAITING.\n\n\n");
			container->update_status(dhtt_msgs::msg::NodeStatus::WAITING);
		}

		to_ret->done = this->is_done();

		return to_ret;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> RootBehavior::work_callback( dhtt::Node* container ) 
	{
		// wow such empty
		(void) container;

		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		return to_ret;
	}

	void RootBehavior::parse_params( std::vector<std::string> params ) 
	{
		if ( (int) params.size() > 2 )
			throw std::invalid_argument("Too many parameters passed to node. Root Behavior only needs path parameter to load the robot resources.");

		if ( (int) params.size() == 0 )
			throw std::invalid_argument("No parameters passed to node. Root Behavior needs path parameter to load the robot resources.");

		auto separator_pos = params[0].find(": ");

		if ( separator_pos == std::string::npos )
			throw std::invalid_argument("Parameters are expected in the format \"key: value\" but received in the form " + params[0] + ". Returning in error.");

		std::string key = params[0].substr(0, separator_pos);
		std::string value = params[0].substr(separator_pos + 2, params[0].size() - separator_pos - 2);

		if ( strcmp(key.c_str(), "path") ) 
			throw std::invalid_argument("Expected parameter path, but received " + key + ". Returning in error.");

		// just meant to remove spaces, could be buggy
		while ( true )
		{
			auto space_finder = value.find(" ");

			if (space_finder == std::string::npos)
				break;

			// should work to remove spaces
			value.replace(space_finder, 1, "");
		}

		this->robot_resources_file_path = value;

		if ( (int) params.size() == 2 )
		{
			this->slow = true;
		}

	}

	double RootBehavior::get_perceived_efficiency(dhtt::Node* container) 
	{
		(void) container;
		return 1.0;
	}

	bool RootBehavior::is_done() 
	{
		return this->children_done;
	}

	void RootBehavior::load_resources_from_yaml()
	{
		// first reset the internal canonical_resources_list
		this->canonical_resources_list.clear();

		// now load the robot resources from a given yaml file (this throws a yaml exception if there is a problem loading the file or interfacing with it)
		YAML::Node config = YAML::LoadFile(this->robot_resources_file_path);
	
		this->robot_name = config["robot_name"].as<std::string>();

		int number_of_resources = config["robot_resources"]["number_of_resources"].as<int>();

		for (int i = 0; i < number_of_resources; i++)
		{
			std::string resource_name = std::string("resource_") + std::to_string(i);

			dhtt_msgs::msg::Resource n_resource;

			n_resource.type = (int8_t) config["robot_resources"][resource_name]["type"].as<int>();

			n_resource.name = config["robot_resources"][resource_name]["name"].as<std::string>();
			n_resource.locked = false;

			this->canonical_resources_list.push_back(n_resource);
		}

		this->push_resources();
	}

	void RootBehavior::publish_resources()
	{
		std::string resources_topic = std::string(TREE_PREFIX) + RESOURCES_POSTFIX;
		dhtt_msgs::msg::Resources n_msg;

		n_msg.resource_state = this->canonical_resources_list;

		this->status_pub->publish(n_msg);
		// this->com_agg->publish_msg<dhtt_msgs::msg::Resources>(resources_topic, n_msg);
	}

	void RootBehavior::push_resources()
	{
		std::vector<rclcpp::Parameter> to_push;

		dhtt_utils::VectorResources data = dhtt_utils::msg_to_vector(this->canonical_resources_list);

		this->com_agg->set_parameters_sync({rclcpp::Parameter("dhtt_resources.names", data.names), 
												rclcpp::Parameter("dhtt_resources.types", data.types),
												rclcpp::Parameter("dhtt_resources.channels", data.channels),
												rclcpp::Parameter("dhtt_resources.locks", data.locks),
												rclcpp::Parameter("dhtt_resources.owners", data.owners)});
		
												
		// DHTT_LOG_FATAL(this->com_agg, "\tCurrent state of resources:");

		// for ( auto iter : this->canonical_resources_list )
		// {
		// 	DHTT_LOG_FATAL(this->com_agg, "\t\t" << iter.name << " - type: " << iter.type << ", locked: " << iter.locked 
		// 		<< ", owners: " << iter.owners << ", channel: " << iter.channel);
		// }
	}

	void RootBehavior::pull_resources()
	{
		auto resource_names = this->com_agg->get_parameter_sync("dhtt_resources.names").as_string_array();
		auto resource_types = this->com_agg->get_parameter_sync("dhtt_resources.types").as_integer_array();
		auto resource_channels = this->com_agg->get_parameter_sync("dhtt_resources.channels").as_integer_array();
		auto resource_locks = this->com_agg->get_parameter_sync("dhtt_resources.locks").as_bool_array();
		auto resource_owners = this->com_agg->get_parameter_sync("dhtt_resources.owners").as_integer_array();

		std::string to_find;
		auto find_resource = [&to_find](dhtt_msgs::msg::Resource to_check){ return not strcmp(to_find.c_str(), to_check.name.c_str()); };

		// add any that we are missing
		for ( int i = 0 ; i < (int) resource_names.size() ; i++ )
		{
			to_find = resource_names[i];
			if ( std::find_if(this->canonical_resources_list.begin(), this->canonical_resources_list.end(), find_resource) == this->canonical_resources_list.end() )
			{
				dhtt_msgs::msg::Resource n_resource;
				n_resource.name = to_find;
				n_resource.type = resource_types[i];
				n_resource.channel = resource_channels[i];
				n_resource.locked = resource_locks[i];
				n_resource.owners = resource_owners[i];

				this->canonical_resources_list.push_back(n_resource);
			}
		}
		
		// now remove any that are missing on the param server
		// if ( resource_names.size() < this->canonical_resources_list.size() )
		// {
		// 	std::vector<int> indices_to_remove;

		// 	for ( int i = 0 ; i < (int) this->canonical_resources_list.size() ; i++ )
		// 		if ( std::find(resource_names.begin(), resource_names.end(), this->canonical_resources_list[i].name) == resource_names.end() )
		// 			indices_to_remove.push_back(i);

		// 	// remove in reverse order so that the indices are always correct
		// 	for ( auto riter = indices_to_remove.rbegin() ; riter != indices_to_remove.rend(); riter++ )
		// 		this->canonical_resources_list.erase(this->canonical_resources_list.begin() + *riter);
		// }
	}

	std::vector<dhtt_msgs::msg::Resource> RootBehavior::give_resources(std::vector<dhtt_msgs::msg::Resource> to_give)
	{
		dhtt_msgs::msg::Resource to_find;

		std::function<bool(dhtt_msgs::msg::Resource)> find_first_compatible_resource = [&]( dhtt_msgs::msg::Resource to_check )
		{
			return (to_find.type == to_check.type) and not to_check.locked;
		};

		std::function<bool(dhtt_msgs::msg::Resource)> find_resource_by_name = [&]( dhtt_msgs::msg::Resource to_check )
		{
			return not strcmp(to_find.name.c_str(), to_check.name.c_str());
		};

		std::vector<dhtt_msgs::msg::Resource> to_ret;

		for ( dhtt_msgs::msg::Resource resource : to_give )
		{
			to_find = resource;
			auto search_lambda = ( to_find.name == "" ) ? find_first_compatible_resource : find_resource_by_name;

			auto found_iter = std::find_if(this->canonical_resources_list.begin(), this->canonical_resources_list.end(), search_lambda);

			if ( found_iter == this->canonical_resources_list.end() )
				throw std::invalid_argument("Could not find resource name [" + to_find.name + "] of type [" + std::to_string(resource.type) + "]. This should have been impossible. Returning in error.");

			DHTT_LOG_DEBUG(this->com_agg, found_iter->name << " given to behavior");

			(*found_iter).locked = true;

			to_ret.push_back(*found_iter);
		}

		if ( to_ret.size() != to_give.size() )
			throw std::runtime_error("Did not find all the required resources!");

		return to_ret;
	}

	void RootBehavior::release_resources(std::vector<dhtt_msgs::msg::Resource> to_release)
	{
		dhtt_msgs::msg::Resource to_find;

		auto find_resource_by_name = [&]( dhtt_msgs::msg::Resource to_check )
		{
			return not strcmp(to_find.name.c_str(), to_check.name.c_str());
		};
		
		for ( dhtt_msgs::msg::Resource resource : to_release )
		{
			to_find = resource;

			auto found_iter = std::find_if(this->canonical_resources_list.begin(), this->canonical_resources_list.end(), find_resource_by_name);

				// throw std::invalid_argument("Could not find resorce with name " + resource.name + ". This should have not have been released. Returning in error.");

			if ( found_iter != this->canonical_resources_list.end() )
				found_iter->locked = false;
		} 
	}
	
	void RootBehavior::add_resources( std::vector<dhtt_msgs::msg::Resource> to_add )
	{
		dhtt_msgs::msg::Resource to_find;

		auto find_resource_by_name = [&]( dhtt_msgs::msg::Resource to_check )
		{
			return not strcmp(to_find.name.c_str(), to_check.name.c_str());
		};

		// first check if any added resources already exist in canonical_resources 
		for ( auto iter : to_add )
		{
			to_find = iter;

			if ( std::find_if(this->canonical_resources_list.begin(), this->canonical_resources_list.end(), find_resource_by_name) != this->canonical_resources_list.end() )
				throw std::runtime_error("Failed to add resource " + iter.name + " because it already exists.");
		}

		// then add the new resources iteratively ( very simple ) and make sure they are added as locked
		for ( auto iter : to_add )
		{
			this->canonical_resources_list.push_back(iter);
			this->canonical_resources_list.back().locked = true;
		}

		return;
	}

	void RootBehavior::remove_resources( std::vector<dhtt_msgs::msg::Resource> to_remove )
	{
		dhtt_msgs::msg::Resource to_find;

		auto find_resource_by_name = [&]( dhtt_msgs::msg::Resource to_check )
		{
			return not strcmp(to_find.name.c_str(), to_check.name.c_str());
		};

		// first make sure all resources exist in canonical resources
		for ( auto iter : to_remove )
		{
			to_find = iter;

			if ( std::find_if(this->canonical_resources_list.begin(), this->canonical_resources_list.end(), find_resource_by_name) == this->canonical_resources_list.end() )
				throw std::runtime_error("Failed to remove resource " + iter.name + " because it does not exist.");
		}

		// then remove them iteratively 
		for ( auto remove_iter : to_remove )
		{
			to_find = remove_iter;

			this->canonical_resources_list.erase(std::find_if(this->canonical_resources_list.begin(), this->canonical_resources_list.end(), find_resource_by_name));
		}

		return;
	}

	void RootBehavior::release_all_resources()
	{
		for (std::vector<dhtt_msgs::msg::Resource>::iterator resource_iter = this->canonical_resources_list.begin(); resource_iter != this->canonical_resources_list.end(); resource_iter++)
		{
			(*resource_iter).locked = false;
			(*resource_iter).channel = dhtt_msgs::msg::Resource::EXCLUSIVE;
		}
	}

	void RootBehavior::control_callback( const std::shared_ptr<dhtt_msgs::srv::InternalControlRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalControlRequest::Response> response )
	{
		if ( request->control_code == dhtt_msgs::srv::InternalControlRequest::Request::GRACEFULSTOP )
		{
			this->interrupted = true;

			response->success = true;
			response->error_msg = "";

			return;
		}

		response->success = false;
		response->error_msg = "Unable to complete request.";

		return;
	}
}