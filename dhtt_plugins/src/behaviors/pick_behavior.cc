#include "dhtt_plugins/behaviors/pick_behavior.hpp"

namespace dhtt_plugins
{
	void PickBehavior::parse_params(std::vector<std::string> params) 
	{
		if ( (int) params.size() > 2 )
			throw std::invalid_argument("Too many parameters passed to node. Only activation potential, and object name required.");

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
		
		separator_pos = params[1].find(": ");

		if ( separator_pos == std::string::npos )
			throw std::invalid_argument("Parameters are expected in the format \"key: value\" but received in the form " + params[1] + ". Returning in error.");

		key = params[1].substr(0, separator_pos);
		value = params[1].substr(separator_pos + 2, params[0].size() - separator_pos); 

		if ( strcmp(key.c_str(), "object") )
			throw std::invalid_argument("Expected parameter object, but received " + key + ". Returning in error.");

		this->object_name = value;
		this->params = params;

	}

	void PickBehavior::do_work( dhtt::Node* container ) 
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
		auto sub_ptr = this->pub_node_ptr->create_subscription<std_msgs::msg::String>("/dhtt/result", 10, std::bind(&PickBehavior::done_callback, this, std::placeholders::_1));

		std_msgs::msg::String go;

		go.data = "any|pick";

		pub_ptr->publish(go);

		this->work_done = false;

		while ( not this->work_done )
			this->executor->spin_once();

		return;
	}

	double PickBehavior::get_perceived_efficiency() 
	{
		return this->activation_potential;
	}

	std::vector<dhtt_msgs::msg::Resource> PickBehavior::get_retained_resources( dhtt::Node* container ) 
	{
		std::vector<dhtt_msgs::msg::Resource> to_ret;

		// just keep access to the first gripper found (shouldn't matter for now)
		for (auto resource_iter : container->get_owned_resources())
		{
			if (resource_iter.type == dhtt_msgs::msg::Resource::GRIPPER)
			{
				to_ret.push_back(resource_iter);
				break;
			}
		}

		return to_ret;
	}
	std::vector<dhtt_msgs::msg::Resource> PickBehavior::get_released_resources( dhtt::Node* container ) 
	{
		std::vector<dhtt_msgs::msg::Resource> to_ret;
		bool first = true;

		for (auto resource_iter : container->get_owned_resources())
		{
			if (resource_iter.type != dhtt_msgs::msg::Resource::GRIPPER or not first)
			{
				to_ret.push_back(resource_iter);
			}
			else
			{// skip the first gripper but not the rest
				first = false;
			}
		}

		return to_ret;
	}

	void PickBehavior::done_callback( std::shared_ptr<std_msgs::msg::String> data )
	{
		(void) data;

		this->work_done = true;
	}
}