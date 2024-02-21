#include "dhtt_plugins/behaviors/action_type.hpp"


namespace dhtt_plugins
{

	void ActionType::initialize(std::vector<std::string> params)
	{
		// these just supress warnings regarding not using function params
		(void) params;

		// only allowed for now until the task logic is implemented
		this->children_allowed = false;
		this->done = false;

		this->parse_params(params);

		return;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> ActionType::auction_callback( dhtt::Node* container )
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

	std::shared_ptr<dhtt_msgs::action::Activation::Result> ActionType::work_callback( dhtt::Node* container )
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

	bool ActionType::is_done()
	{
		return this->done;
	}
}