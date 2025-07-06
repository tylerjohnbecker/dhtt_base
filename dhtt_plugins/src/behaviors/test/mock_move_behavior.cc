#include "dhtt_plugins/behaviors/test/mock_move_behavior.hpp"

namespace dhtt_plugins
{
	void MockMoveBehavior::parse_params(std::vector<std::string> params) 
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
		value = params[1].substr(separator_pos + 2, params[1].size() - separator_pos); 

		if ( strcmp(key.c_str(), "dest") )
			throw std::invalid_argument("Expected parameter dest, but received " + key + ". Returning in error.");

		this->destination = value;
		this->params = params;

		this->done = false;
	}

	void MockMoveBehavior::do_work( dhtt::Node* container ) 
	{
		(void) container;

		// change the robot's position in the param server
		std::shared_ptr<rclcpp::Node> param_node_ptr = std::make_shared<rclcpp::Node>(container->get_node_name() + "_param_getter");

		auto params_client_ptr = std::make_shared<rclcpp::SyncParametersClient>(param_node_ptr, "/param_node");
		{
			using namespace std::chrono_literals;

			while ( not params_client_ptr->wait_for_service(1s) and rclcpp::ok() ); 
		};

		std::string robot_loc = params_client_ptr->get_parameter<std::string>("world.robot.location");

		RCLCPP_FATAL(container->get_logger(), "\t\t\tInitial robot location [%s]", robot_loc.c_str());

		params_client_ptr->set_parameters({rclcpp::Parameter("world.robot.location", this->destination)});

		robot_loc = params_client_ptr->get_parameter<std::string>("world.robot.location");

		RCLCPP_FATAL(container->get_logger(), "\t\t\t after update robot location [%s]", robot_loc.c_str());

		// fire a knowledge update message
		this->send_state_updated();

		this->done = true;
	}

	std::vector<dhtt_msgs::msg::Resource> MockMoveBehavior::get_retained_resources( dhtt::Node* container ) 
	{
		return container->get_owned_resources();
	}

	std::vector<dhtt_msgs::msg::Resource> MockMoveBehavior::get_released_resources( dhtt::Node* container ) 
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}

	std::vector<dhtt_msgs::msg::Resource> MockMoveBehavior::get_necessary_resources() 
	{
		std::vector<dhtt_msgs::msg::Resource> to_ret;

		dhtt_msgs::msg::Resource base;
		base.type = dhtt_msgs::msg::Resource::BASE;

		to_ret.push_back(base);

		return to_ret;
	}

	double MockMoveBehavior::get_perceived_efficiency(dhtt::Node* container) 
	{
		(void) container; 
		
		return this->activation_potential;
	}
}