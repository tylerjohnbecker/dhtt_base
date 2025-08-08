#include "dhtt_plugins/tasks/root_behavior.hpp"

namespace dhtt_plugins
{
	void RootBehavior::initialize(std::vector<std::string> params)
	{
		this->pub_node_ptr = std::make_shared<rclcpp::Node>("dhtt_root");

		std::string resources_topic = std::string(TREE_PREFIX) + RESOURCES_POSTFIX;
		std::string control_topic = std::string(TREE_PREFIX) + CONTROL_POSTFIX;	

		this->status_pub = this->com_agg->register_publisher<dhtt_msgs::msg::Resources>(resources_topic);
		// this->status_pub = this->pub_node_ptr->create_publisher<dhtt_msgs::msg::Resources>(resources_topic, 10);//

		this->control_server = this->pub_node_ptr->create_service<dhtt_msgs::srv::InternalControlRequest>(control_topic, std::bind(&RootBehavior::control_callback, this, std::placeholders::_1, std::placeholders::_2));

		// this->param_client_ptr = this->com_agg->register_param_client("/param_node");

		this->resource_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
		this->resource_executor->add_node(this->pub_node_ptr);

		this->children_done = false;

		this->children_allowed = true;
		this->interrupted = false;

		this->params = params;
		this->slow = false;
		this->parse_params(params);
		this->load_resources_from_yaml();

		this->spin_thread = std::make_shared<std::thread>(&RootBehavior::async_spin, this);
		this->spin_thread->detach();
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> RootBehavior::auction_callback( dhtt::Node* container ) 
	{
		auto print_resource_state = [&]()
		{
			DHTT_LOG_FATAL(this->com_agg, "\tCurrent state of resources:");

			for ( auto iter : this->canonical_resources_list )
			{
				DHTT_LOG_FATAL(this->com_agg, "\t\t" << iter.name << " - type: " << iter.type << ", locked: " << iter.locked 
					<< ", owners: " << iter.owners << ", channel: " << iter.channel);
			}
		};

		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		DHTT_LOG_INFO(this->com_agg, "\n\n\n\tActivation received starting task execution...");

		this->children_done = false;
		this->interrupted = false;

		// this relies on there only being one direct child of the root node (should enforce this as well)
		while ( not this->is_done() and not this->interrupted )
		{

			// publish state of resources to all nodes in the tree
			// this->publish_resources();
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
			DHTT_LOG_FATAL(this->com_agg, "\tRequest complete, releasing resources!\n\n --- --- ---");

			this->release_resources( (*result.begin()).second.released_resources );
			this->release_resources( (*result.begin()).second.passed_resources );
			// print_resource_state();

			this->children_done = (*result.begin()).second.done;

			if (this->slow)
				rclcpp::sleep_for(std::chrono::milliseconds(100));
		}
		
		this->release_all_resources();

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

		// now load the robot resources from a given yaml file (this throws a yaml exception if there is a problem loading the file or interfacing with it)
		YAML::Node config = YAML::LoadFile(this->robot_resources_file_path);
	
		this->robot_name = config["robot_name"].as<std::string>();

		int number_of_resources = config["robot_resources"]["number_of_resources"].as<int>();

		for (int i = 0; i < number_of_resources; i++)
		{
			std::string resource_name = std::string("resource_") + std::to_string(i);

			dhtt_msgs::msg::Resource n_resource;

			n_resource.type = config["robot_resources"][resource_name]["type"].as<int>();
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

		std::vector<std::string> names;
		std::vector<int> types;
		std::vector<int> channels;
		std::vector<bool> locks;
		std::vector<int> owners;

		//use the internal param server to update resources
		for ( auto iter : this->canonical_resources_list )
		{
			names.push_back(iter.name);
			types.push_back(iter.type);
			channels.push_back(iter.channel);
			locks.push_back(iter.locked);
			owners.push_back(iter.owners);
		}

		this->com_agg->set_parameters_sync({rclcpp::Parameter("dhtt_resources.names", names), 
												rclcpp::Parameter("dhtt_resources.types", types),
												rclcpp::Parameter("dhtt_resources.channels", channels),
												rclcpp::Parameter("dhtt_resources.locks", locks),
												rclcpp::Parameter("dhtt_resources.owners", owners)});
	}

	std::vector<dhtt_msgs::msg::Resource> RootBehavior::give_resources(std::vector<dhtt_msgs::msg::Resource> to_give)
	{
		dhtt_msgs::msg::Resource to_find;

		auto find_first_compatible_resource = [&]( dhtt_msgs::msg::Resource to_check )
		{
			return (to_find.type == to_check.type) and not to_check.locked;
		};

		std::vector<dhtt_msgs::msg::Resource> to_ret;

		for ( dhtt_msgs::msg::Resource resource : to_give )
		{
			to_find = resource;

			auto found_iter = std::find_if(this->canonical_resources_list.begin(), this->canonical_resources_list.end(), find_first_compatible_resource);

			if ( found_iter == this->canonical_resources_list.end() )
				throw std::invalid_argument("Could not find resource of type " + std::to_string(resource.type) + ". This should have been marked impossible. Returning in error.");

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

			if ( found_iter == this->canonical_resources_list.end() )
				throw std::invalid_argument("Could not find resorce with name " + resource.name + ". This should have not have been released. Returning in error.");

			(*found_iter).locked = false;
		} 
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

	void RootBehavior::async_spin()
	{
		while ( rclcpp::ok() )
			this->resource_executor->spin_once();
	}
}