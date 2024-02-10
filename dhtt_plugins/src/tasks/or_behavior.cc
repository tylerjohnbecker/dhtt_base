#include "dhtt_plugins/tasks/or_behavior.hpp"

namespace dhtt_plugins
{
	void OrBehavior::initialize(std::vector<std::string> params) 
	{

	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> OrBehavior::auction_callback( dhtt::Node* container ) 
	{
		return std::shared_ptr<dhtt_msgs::action::Activation::Result>();
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> OrBehavior::work_callback( dhtt::Node* container, bool success) 
	{
		return std::shared_ptr<dhtt_msgs::action::Activation::Result>();
	}

	void OrBehavior::parse_params( std::vector<std::string> params ) 
	{

	}

	double OrBehavior::get_perceived_efficiency() 
	{
		return this->activation_potential;
	}

	std::vector<dhtt_msgs::msg::Resource> OrBehavior::get_retained_resources( dhtt::Node* container ) 
	{
		return std::vector<dhtt_msgs::msg::Resource>();
	}

	std::vector<dhtt_msgs::msg::Resource> OrBehavior::get_released_resources( dhtt::Node* container ) 
	{
		return std::vector<dhtt_msgs::msg::Resource>();
	}

	bool OrBehavior::isDone() 
	{
		return false;
	}

}