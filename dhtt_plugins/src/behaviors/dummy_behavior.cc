#include "dhtt_plugins/behaviors/dummy_behavior.hpp"


namespace dhtt_plugins
{

	void DummyBehavior::initialize(std::vector<std::string> params)
	{
		return;
	};

	void DummyBehavior::auction_callback( dhtt::Node& container )
	{
		// doesn't need any functionality till the rest of the tree is made, but just needs to send up an empty request with activation potential
	};

	void DummyBehavior::result_callback( dhtt::Node& container, bool success)
	{
		// not sure what goes here for now, work will be called from Node
	};

	void DummyBehavior::work()
	{
		// do work ofc

		// constructor for rate take hz as an input
		rclcpp::Rate fake_work(.5);

		fake_work.sleep();
	};

	double DummyBehavior::get_perceived_efficiency()
	{
		// just give random perceived efficiency
		return ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) );
	};

	bool DummyBehavior::can_add_child()
	{
		return false;
	};

	std::vector<dhtt_msgs::msg::Resource> DummyBehavior::get_retained_resources( dhtt::Node& container )
	{
		return std::vector<dhtt_msgs::msg::Resource>();
	};

	std::vector<dhtt_msgs::msg::Resource> DummyBehavior::get_released_resources( dhtt::Node& container )
	{
		return container.get_owned_resources();
	};
};