#include "dhtt_plugins/behaviors/place_behavior.hpp"

namespace dhtt_plugins
{
	void PlaceBehavior::parse_params(std::vector<std::string> params) 
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
			throw std::invalid_argument("Expected parameter activation_potential, but received " + key + ". Returning in error.");

		this->activation_potential = atof(value.c_str());
		this->params = params;

	}

	void PlaceBehavior::do_work( dhtt::Node* container ) 
	{
		auto resources = container->get_owned_resources();

		bool left = false;

		for (auto resource_iter : resources)
		{
			if (resource_iter.type == dhtt_msgs::msg::Resource::GRIPPER)
			{
				left = not strcmp(resource_iter.name.c_str(), "left_arm");
			}
		}

		auto pub_ptr = this->pub_node_ptr->create_publisher<std_msgs::msg::String>((left)? "/dhtt/left_arm" : "/dhtt/right_arm", 10);
		auto sub_ptr = this->pub_node_ptr->create_subscription<std_msgs::msg::String>("/dhtt/result", 10, std::bind(&PlaceBehavior::done_callback, this, std::placeholders::_1));

		std_msgs::msg::String go;

		go.data = "any|place";

		pub_ptr->publish(go);

		this->work_done = false;

		while ( not this->work_done )
			this->executor->spin_once();

		return;
	}

	double PlaceBehavior::get_perceived_efficiency() 
	{
		return this->activation_potential;
	}	

	std::vector<dhtt_msgs::msg::Resource> PlaceBehavior::get_retained_resources( dhtt::Node* container ) 
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}
	std::vector<dhtt_msgs::msg::Resource> PlaceBehavior::get_released_resources( dhtt::Node* container ) 
	{
		return container->get_owned_resources();
	}

	void PlaceBehavior::done_callback( std::shared_ptr<std_msgs::msg::String> data )
	{
		(void) data;

		this->work_done = true;
	}
}