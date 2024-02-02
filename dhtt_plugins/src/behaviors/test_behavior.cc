#include "dhtt_plugins/behaviors/test_behavior.hpp"


namespace dhtt_plugins
{

	void TestBehavior::initialize(std::vector<std::string> params)
	{
		// these just supress warnings regarding not using function params
		(void) params;

		// only allowed for now until the task logic is implemented
		this->children_allowed = true;

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
		return ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) );
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