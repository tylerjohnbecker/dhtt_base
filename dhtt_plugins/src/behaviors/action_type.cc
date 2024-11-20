#include "dhtt_plugins/behaviors/action_type.hpp"


namespace dhtt_plugins
{

	void ActionType::initialize(std::vector<std::string> params)
	{
		// these just supress warnings regarding not using function params
		(void) params;

		// just a random hopefully unique name
		const void * address = static_cast<const void*>(this);
		std::stringstream ss;
		ss << "action_" << address;

		this->pub_node_ptr = std::make_shared<rclcpp::Node>(ss.str().c_str());

		// only allowed for now until the task logic is implemented
		this->children_allowed = false;
		this->done = false;

		this->parse_params(params);

		this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
		this->executor->add_node(this->pub_node_ptr);

		this->necessary_resources = this->get_necessary_resources();

		this->knowledge_pub = this->pub_node_ptr->create_publisher<std_msgs::msg::String>("/updated_knowledge", 10);

		return;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> ActionType::auction_callback( dhtt::Node* container )
	{
		RCLCPP_INFO(container->get_logger(), "\tActivating and sending back request...");

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

		to_ret->requested_resources = necessary_resources_cp;
		to_ret->owned_resources = container->get_owned_resources();
		to_ret->done = this->is_done();
		to_ret->possible = container->is_request_possible(to_ret->requested_resources);

		return to_ret;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> ActionType::work_callback( dhtt::Node* container )
	{
		// not sure what goes here for now, work will be called from Node
		(void) container;
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		RCLCPP_INFO(container->get_logger(), "\tPerforming work");

		this->do_work(container);

		to_ret->done = this->is_done();
		to_ret->released_resources = this->get_released_resources(container);
		to_ret->passed_resources = this->get_retained_resources(container);

		to_ret->last_behavior = container->get_node_name();
		to_ret->success = this->is_done();

		return to_ret;
	}

	bool ActionType::is_done()
	{
		return this->done;
	}

	void ActionType::send_state_updated()
	{
		std_msgs::msg::String n_msg;

		this->knowledge_pub->publish(n_msg);
	}
}