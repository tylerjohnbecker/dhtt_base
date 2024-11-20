#include "dhtt_plugins/behaviors/test/mock_look_behavior.hpp"

namespace dhtt_plugins
{
	void MockLookBehavior::parse_params(std::vector<std::string> params) 
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

		if ( strcmp(key.c_str(), "object") )
			throw std::invalid_argument("Expected parameter object, but received " + key + ". Returning in error.");

		this->object_target = value;
		this->params = params;
	}

	void MockLookBehavior::do_work( dhtt::Node* container ) 
	{
		(void) container;
		
		// check the robot's location for any found objects
		std::shared_ptr<rclcpp::Node> param_node_ptr = std::make_shared<rclcpp::Node>(container->get_node_name() + "_param_getter");

		auto params_client_ptr = std::make_shared<rclcpp::SyncParametersClient>(param_node_ptr, "/param_node");
		{
			using namespace std::chrono_literals;

			while ( not params_client_ptr->wait_for_service(1s) and rclcpp::ok() ); 
		};

		std::string robot_location = params_client_ptr->get_parameter<std::string>("world.robot.location");

		int num_objects = params_client_ptr->get_parameter<int>("world.objects.num_objects");

		std::string object_lhs = "world.objects.";

		this->done = false;

		RCLCPP_FATAL(container->get_logger(), "Looking for object [%s]", this->object_target.c_str());

		for (int i = 0; i < num_objects; i++)
		{
			std::string object_name = "object_" + std::to_string(i);
			
			std::string param_object_name = object_lhs + object_name;

			std::string object_location = params_client_ptr->get_parameter<std::string>(param_object_name + ".location_gt");

			// check that the robot and the object are at the same spot
			if ( not strcmp(robot_location.c_str(), object_location.c_str()) )
			{
				params_client_ptr->set_parameters( {rclcpp::Parameter(param_object_name + ".location", robot_location)} );
				params_client_ptr->set_parameters( {rclcpp::Parameter(param_object_name + ".probability", 1.0)} );

				RCLCPP_FATAL(container->get_logger(), "Found object [%s] at location [%s].", object_name.c_str(), robot_location.c_str());

				// this behavior is done/successful if the target object was found
				if ( not strcmp( this->object_target.c_str(), object_name.c_str() ) )
					this->done = true;

			}
		}

		if ( this->done )
			RCLCPP_FATAL(container->get_logger(), "Found object!");
		else
			RCLCPP_FATAL(container->get_logger(), "Object not found!");

		// fire a knowledge update message
		this->send_state_updated();
	}

	std::vector<dhtt_msgs::msg::Resource> MockLookBehavior::get_retained_resources( dhtt::Node* container ) 
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}

	std::vector<dhtt_msgs::msg::Resource> MockLookBehavior::get_released_resources( dhtt::Node* container ) 
	{
		return container->get_owned_resources();
	}

	std::vector<dhtt_msgs::msg::Resource> MockLookBehavior::get_necessary_resources() 
	{
		std::vector<dhtt_msgs::msg::Resource> to_ret;

		dhtt_msgs::msg::Resource base;
		base.type = dhtt_msgs::msg::Resource::BASE;

		to_ret.push_back(base);

		return to_ret;
	}

	double MockLookBehavior::get_perceived_efficiency() 
	{
		return this->activation_potential;
	}
}