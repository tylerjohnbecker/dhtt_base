#include "dhtt_plugins/behaviors/move_behavior.hpp"

namespace dhtt_plugins
{
	void MoveBehavior::parse_params(std::vector<std::string> params) 
	{
		if ( (int) params.size() > 2 )
			throw std::invalid_argument("Too many parameters passed to node. Only activation potential, and destination required.");

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

		// check if activation potential was left blank
		float temp = atof(value.c_str()); 

		if ( temp > 0 )
			this->activation_potential = temp;
		
		separator_pos = params[1].find(": ");

		if ( separator_pos == std::string::npos )
			throw std::invalid_argument("Parameters are expected in the format \"key: value\" but received in the form " + params[1] + ". Returning in error.");

		key = params[1].substr(0, separator_pos);
		value = params[1].substr(separator_pos + 2, params[0].size() - separator_pos); 

		if ( strcmp(key.c_str(), "dest") )
			throw std::invalid_argument("Expected parameter dest, but received " + key + ". Returning in error.");

		this->destination = value;
		this->params = params;
	}

	void MoveBehavior::do_work( dhtt::Node* container ) 
	{
		this->com_agg->register_subscription<std_msgs::msg::String>("/dhtt/result", container->get_node_name(), std::bind(&MoveBehavior::done_callback, this, std::placeholders::_1));

		(void) container;

		// for ( int i = 0; i < 10 ; i++ )
		{
			// create publisher in scope to save space
			auto pub_ptr = this->com_agg->register_publisher<std_msgs::msg::String>("/dhtt/move_base");

			std_msgs::msg::String go;

			go.data = this->destination + "|false";

			pub_ptr->publish(go);
		}

		this->work_done = false;

		// needs a refactor anyways so just commenting out for now
		// while ( not this->work_done )
		// 	this->com_agg->spin_some();

		this->done = true;

		// unregister subscriber after we no longer need it
		this->com_agg->unregister_subscription<std_msgs::msg::String>("/dhtt/result", container->get_node_name());

		return;

	}

	double MoveBehavior::get_perceived_efficiency(dhtt::Node* container) 
	{
		(void) container; 
		
		return this->activation_potential;
	}

	std::vector<dhtt_msgs::msg::Resource> MoveBehavior::get_retained_resources( dhtt::Node* container ) 
	{
		return container->get_owned_resources();
	}
	std::vector<dhtt_msgs::msg::Resource> MoveBehavior::get_released_resources( dhtt::Node* container ) 
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}

	std::vector<dhtt_msgs::msg::Resource> MoveBehavior::get_necessary_resources()
	{
		std::vector<dhtt_msgs::msg::Resource> to_ret;

		dhtt_msgs::msg::Resource base;
		base.type = dhtt_msgs::msg::Resource::BASE;

		to_ret.push_back(base);

		return to_ret;
	}

	void MoveBehavior::done_callback( std::shared_ptr<std_msgs::msg::String> data )
	{
		(void) data;

		std::cout << "RECEIVED DONE" << std::endl;

		this->work_done = true;
	}
}