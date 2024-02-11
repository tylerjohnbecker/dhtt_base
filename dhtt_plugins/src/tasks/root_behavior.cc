#include "dhtt_plugins/tasks/root_behavior.hpp"

namespace dhtt_plugins
{
	void RootBehavior::initialize(std::vector<std::string> params)
	{
		this->parse_params(params);

		this->load_resources_from_yaml();

		this->pub_node_ptr = std::make_shared<rclcpp::Node>("dhtt_root");

		std::string resources_topic = std::string(TREE_PREFIX) + RESOURCES_POSTFIX;

		this->status_pub = this->pub_node_ptr->create_publisher<dhtt_msgs::msg::Resources>(resources_topic, 10);

		this->resource_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
		this->resource_executor->add_node(this->pub_node_ptr);

		this->children_done = false;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> RootBehavior::auction_callback( dhtt::Node* container ) 
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		// this relies on there only being one direct child of the root node (should enforce this as well)
		while ( not this->isDone() )
		{
			// publish state of resources to all nodes in the tree
			this->publish_resources();

			// activate children
			dhtt_msgs::action::Activation::Goal n_goal;

			container->activate_all_children(n_goal);

			// get responses
			container->block_for_responses_from_children();

			auto result = container->get_activation_results();

			if ( not (*result.begin()).second->possible )
			{
				container->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

				to_ret->done = false;

				return to_ret;
			}

			// update resources
			n_goal.granted_resources = this->give_resources( (*result.begin()).second->requested_resources );

			// activate children
			container->activate_all_children(n_goal);

			// get responses
			container->block_for_responses_from_children();

			result = container->get_activation_results();

			// update resources
			this->release_resources( (*result.begin()).second->released_resources );

			this->children_done = (*result.begin()).second->done;

		}

		container->update_status(dhtt_msgs::msg::NodeStatus::DONE);

		to_ret->done = true;

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
		if ( (int) params.size() > 1 )
			throw std::invalid_argument("Too many parameters passed to node. Root Behavior only needs path parameter to load the robot resources.");

		if ( (int) params.size() == 0 )
			throw std::invalid_argument("No parameters passed to node. Root Behavior needs path parameter to load the robot resources.");

		auto separator_pos = params[0].find(": ");

		if ( separator_pos == std::string::npos )
			throw std::invalid_argument("Parameters are expected in the format \"key: value\" but received in the form " + params[0] + ". Returning in error.");

		std::string key = params[0].substr(0, separator_pos);
		std::string value = params[0].substr(separator_pos, params[0].size() - separator_pos);

		if ( strcmp(key.c_str(), "path") ) 
			throw std::invalid_argument("Expected parameter path, but received " + key + ". Returning in error.");

		int space_finder = 0;

		while ( true )
		{
			space_finder = value.find(" ");

			if (space_finder >= (int)value.size())
				break;

			// should work to remove spaces
			value.replace(space_finder, 1, "");
		}

		this->robot_resources_file_path = value;
	}

	double RootBehavior::get_perceived_efficiency() 
	{
		return 1.0;
	}

	std::vector<dhtt_msgs::msg::Resource> RootBehavior::get_retained_resources( dhtt::Node* container ) 
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}

	std::vector<dhtt_msgs::msg::Resource> RootBehavior::get_released_resources( dhtt::Node* container ) 
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}

	bool RootBehavior::isDone() 
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
	}

	void RootBehavior::publish_resources()
	{
		dhtt_msgs::msg::Resources n_msg;

		n_msg.resource_state = this->canonical_resources_list;

		this->status_pub->publish(n_msg);
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
				throw std::invalid_argument("Could not find resorce of type " + std::to_string(resource.type) + ". This should have been marked impossible. Returning in error.");

			(*found_iter).locked = true;

			to_ret.push_back(*found_iter);
		}

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
}