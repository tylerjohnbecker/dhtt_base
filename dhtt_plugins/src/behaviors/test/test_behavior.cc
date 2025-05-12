#include "dhtt_plugins/behaviors/test/test_behavior.hpp"


namespace dhtt_plugins
{

	void TestBehavior::initialize(std::vector<std::string> params)
	{
		// these just supress warnings regarding not using function params
		(void) params;

		// only allowed for now until the task logic is implemented
		this->children_allowed = false;
		this->done = false;

		this->parse_params(params);

		dhtt_msgs::msg::Resource gripper;
		gripper.type = dhtt_msgs::msg::Resource::GRIPPER;

		this->necessary_resources.push_back(gripper);

		dhtt_msgs::msg::Resource head;
		head.type = dhtt_msgs::msg::Resource::HEAD;
		
		this->necessary_resources.push_back(head);
		
		dhtt_msgs::msg::Resource base;
		base.type = dhtt_msgs::msg::Resource::BASE;
		
		this->necessary_resources.push_back(base);

		return;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> TestBehavior::auction_callback( dhtt::Node* container )
	{
		RCLCPP_INFO(container->get_logger(), "\tActivating and sending back request...");

		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		to_ret->local_best_node = container->get_node_name();

		// removed owned resources from the list of requested resources
		dhtt_msgs::msg::Resource cur_resource;
		auto find_resource = [&]( dhtt_msgs::msg::Resource to_check ){ return to_check.type == cur_resource.type; };

		// make a deep copy of the necessary resource and then remove owned ones from the copy
		std::vector<dhtt_msgs::msg::Resource> necessary_resources_cp;

		for ( auto resource : this->necessary_resources )
			necessary_resources_cp.push_back(resource);

		RCLCPP_DEBUG(container->get_logger(), "Collecting owned resources and sending request for necessary resources");

		for ( auto resource : container->get_owned_resources() )
		{
			cur_resource = resource;
			auto found = std::find_if( necessary_resources_cp.begin(), necessary_resources_cp.end(), find_resource);

			if ( found != necessary_resources_cp.end() )
				necessary_resources_cp.erase(found);
		}

		to_ret->requested_resources = necessary_resources_cp;
		to_ret->owned_resources = container->get_owned_resources();
		to_ret->done = this->is_done();

		RCLCPP_DEBUG(container->get_logger(), "Checking if task is possible given current resource state.");

		bool is_possible = true;

		auto resources = container->get_resource_state();

		for ( auto resource : to_ret->requested_resources )
		{
			cur_resource = resource;
			auto found = std::find_if(resources.begin(), resources.end(), find_resource);

			while ( found != resources.end() )
			{
				if ((*found).locked == false )
					break;
				else
					found = std::find_if(found + 1, resources.end(), find_resource);
			}

			if ( found == resources.end() )
			{
				is_possible = false;
				break;
			}
		}

		if ( not is_possible )
			RCLCPP_WARN(container->get_logger(), "\tNecessary resources unavailable.");

		to_ret->possible = is_possible;

		return to_ret;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> TestBehavior::work_callback( dhtt::Node* container )
	{
		// not sure what goes here for now, work will be called from Node
		(void) container;
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		RCLCPP_INFO(container->get_logger(), "\tPerforming work...");

		this->done = true;

		to_ret->done = this->is_done();

		std::vector<dhtt_msgs::msg::Resource> passed;
		std::vector<dhtt_msgs::msg::Resource> released;

		bool first = true;
		
		for (dhtt_msgs::msg::Resource resource : container->get_owned_resources())
		{
			if (resource.type == dhtt_msgs::msg::Resource::GRIPPER and first)
			{
				passed.push_back(resource);
				RCLCPP_DEBUG(container->get_logger(), "Passing control of resource %s", resource.name.c_str());

				first = false;
			}
			else
			{
				released.push_back(resource);
				RCLCPP_DEBUG(container->get_logger(), "Releasing control of resource %s", resource.name.c_str());
			}
		}

		to_ret->passed_resources = passed;
		to_ret->released_resources = released;
		return to_ret;
	}

	void TestBehavior::parse_params( std::vector<std::string> params )
	{
		// if ( (int) params.size() > 2 )
		// 	throw std::invalid_argument("Too many parameters passed to node. Only activation potential required.");

		if ( (int) params.size() == 0 )
		{
			this->activation_potential = ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) );

			return;
		}

		auto separator_pos = params[0].find(": ");

		if ( separator_pos == std::string::npos )
			throw std::invalid_argument("Parameters are expected in the format \"key: value\" but received in the form " + params[0] + ". Returning in error.");

		std::string key = params[0].substr(0, separator_pos);
		std::string value = params[0].substr(separator_pos + 2, params[0].size() - separator_pos); 

		if ( strcmp(key.c_str(), "activation_potential") )
			throw std::invalid_argument("Class TestBehavior only expects parameter activation_potential, but received " + key + ". Returning in error.");

		this->activation_potential = atof(value.c_str());

		if ( (int) params.size() >= 2 )
		{
			for ( int i = 0; i < (int) params.size() - 1; i++)
			{
				std::string check_negate = params[1];
				bool negate = false;

				auto negate_pos = check_negate.find("!");

				if ( negate_pos != std::string::npos )
				{
					negate = true;
					check_negate = check_negate.substr(negate_pos + 1, check_negate.size() - 1);
				}

				auto separator_pos = check_negate.find(": ");

				if ( separator_pos == std::string::npos )
					throw std::invalid_argument("Parameters are expected in the format \"key: value\" but received in the form " + params[1] + ". Returning in error.");

				key = check_negate.substr(0, separator_pos);
				value = check_negate.substr(separator_pos + 2, check_negate.size() - separator_pos); 

				dhtt_msgs::msg::Pair n_pre;

				n_pre.key = key;
				n_pre.value = value;
				n_pre.negate = negate;
				n_pre.type = dhtt_msgs::msg::Pair::LOCATION;

				dhtt_msgs::msg::Pair n_post;

				n_post.key = key;
				n_post.value = value;
				n_post.negate = not negate;
				n_post.type = dhtt_msgs::msg::Pair::LOCATION;

				this->preconditions.predicates.push_back(n_pre);
				this->postconditions.predicates.push_back(n_post);

				this->preconditions.logical_operator = dhtt_utils::LOGICAL_AND;
				this->postconditions.logical_operator = dhtt_utils::LOGICAL_AND;
			}
		}

		this->params = params;
	}

	double TestBehavior::get_perceived_efficiency()
	{
		// just give random perceived efficiency
		return this->activation_potential;
	}

	bool TestBehavior::is_done()
	{
		return this->done;
	}
}