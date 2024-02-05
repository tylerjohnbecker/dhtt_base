#include "dhtt_plugins/behaviors/test_behavior.hpp"


namespace dhtt_plugins
{

	void TestBehavior::initialize(std::vector<std::string> params)
	{
		// these just supress warnings regarding not using function params
		(void) params;

		// only allowed for now until the task logic is implemented
		this->children_allowed = true;

		this->parse_params(params);

		return;
	}

	void TestBehavior::auction_callback( dhtt::Node& container )
	{
		// doesn't need any functionality till the rest of the tree is made, but just needs to send up an empty request with activation potential

		(void) container;
	}

	void TestBehavior::result_callback( dhtt::Node& container, bool success)
	{
		// not sure what goes here for now, work will be called from Node
		(void) container;
		(void) success;
	}

	void TestBehavior::parse_params( std::vector<std::string> params )
	{
		if ( (int) params.size() > 1 )
			throw std::invalid_argument("Too many parameters passed to node. Only activation potential required.");

		if ( (int) params.size() == 0 )
			this->activation_potential = ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) );

		auto separator_pos = params[0].find(": ");

		if ( separator_pos == std::string::npos )
			throw std::invalid_argument("Parameters are expected in the format \"key: value\" but received in the form " + params[0] + ". Returning in error.");

		std::string key = params[0].substr(0, separator_pos);
		std::string value = params[0].substr(separator_pos, params[0].size() - separator_pos); 

		if ( strcmp(key.c_str(), "activation_potential") )
			throw std::invalid_argument("Class TestBehavior only expects parameter activation_potential, but received " + key + ". Returning in error.");

		this->activation_potential = atof(value.c_str());
	}

	void TestBehavior::work()
	{
		// do work ofc

		// constructor for rate take hz as an input
		rclcpp::Rate fake_work(.5);

		fake_work.sleep();
	}

	double TestBehavior::get_perceived_efficiency()
	{
		// just give random perceived efficiency
		return this->activation_potential;
	}

	std::vector<dhtt_msgs::msg::Resource> TestBehavior::get_retained_resources( dhtt::Node& container )
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}

	std::vector<dhtt_msgs::msg::Resource> TestBehavior::get_released_resources( dhtt::Node& container )
	{
		(void) container;

		return container.get_owned_resources();
	}
}