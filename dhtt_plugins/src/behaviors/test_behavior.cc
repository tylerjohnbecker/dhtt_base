#include "dhtt_plugins/behaviors/test_behavior.hpp"


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

		return;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> TestBehavior::auction_callback( dhtt::Node* container )
	{
		RCLCPP_INFO(container->get_logger(), "Activating and sending back request...");

		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		to_ret->local_best_node = container->get_node_name();

		// removed owned resources from the list of requested resources
		dhtt_msgs::msg::Resource cur_resource;
		auto find_resource = [&]( dhtt_msgs::msg::Resource to_check ){ return to_check.type == cur_resource.type; };

		// make a deep copy of the necessary resource and then remove owned ones from the copy
		std::vector<dhtt_msgs::msg::Resource> necessary_resources_cp;

		for ( auto resource : this->necessary_resources )
			necessary_resources_cp.push_back(resource);

		for ( auto resource : container->get_owned_resources() )
		{
			cur_resource = resource;
			auto found = std::find_if( necessary_resources_cp.begin(), necessary_resources_cp.end(), find_resource);

			if ( found != necessary_resources_cp.end() )
				necessary_resources_cp.erase(found);
		}

		to_ret->requested_resources = necessary_resources;
		to_ret->owned_resources = container->get_owned_resources();
		to_ret->done = this->is_done();

		bool is_possible = true;

		auto resources = container->get_resource_state();

		for ( auto resource : resources )
		{
			cur_resource = resource;
			auto found = std::find_if(resources.begin(), resources.end(), find_resource);

			while ( found != resources.end() )
			{
				if ((*found).locked == false )
					break;
				else
					found = std::find_if(found, resources.end(), find_resource);
			}

			if ( found == resources.end() )
			{
				is_possible = false;
				break;
			}
		}

		to_ret->possible = is_possible;

		return to_ret;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> TestBehavior::work_callback( dhtt::Node* container )
	{
		// not sure what goes here for now, work will be called from Node
		(void) container;
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		RCLCPP_INFO(container->get_logger(), "Performing work for [%s]", container->get_node_name().c_str());

		this->done = true;

		to_ret->done = this->is_done();
		to_ret->released_resources = this->get_released_resources(container);
		to_ret->passed_resources = this->get_retained_resources(container);

		return to_ret;
	}

	void TestBehavior::parse_params( std::vector<std::string> params )
	{
		if ( (int) params.size() > 1 )
			throw std::invalid_argument("Too many parameters passed to node. Only activation potential required.");

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
	}

	double TestBehavior::get_perceived_efficiency()
	{
		// just give random perceived efficiency
		return this->activation_potential;
	}

	std::vector<dhtt_msgs::msg::Resource> TestBehavior::get_retained_resources( dhtt::Node* container )
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}

	std::vector<dhtt_msgs::msg::Resource> TestBehavior::get_released_resources( dhtt::Node* container )
	{
		(void) container;

		return container->get_owned_resources();
	}

	bool TestBehavior::is_done()
	{
		return this->done;
	}
}