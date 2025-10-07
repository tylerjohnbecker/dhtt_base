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

		// only allowed for now until the task logic is implemented
		this->children_allowed = false;
		this->done = false;

		this->necessary_resources = this->get_necessary_resources();

		this->parse_params(params);
		return;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> ActionType::auction_callback( dhtt::Node* container )
	{
		DHTT_LOG_INFO(this->com_agg, "\tActivating and sending back request...");

		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		to_ret->local_best_node = container->get_node_name();

		// make sure the world resource lists are up to date
		this->populate_resource_lists(container);

		// removed owned resources from the list of requested resources
		dhtt_msgs::msg::Resource cur_resource;
		std::function<bool(dhtt_msgs::msg::Resource)> find_resource_by_type = [&cur_resource]( dhtt_msgs::msg::Resource to_check ){ return to_check.type == cur_resource.type; };
		std::function<bool(dhtt_msgs::msg::Resource)> find_resource_by_name = [&cur_resource]( dhtt_msgs::msg::Resource to_check ){ return not strcmp( to_check.name.c_str(), cur_resource.name.c_str() );  };

		// make a deep copy of the necessary resource and then remove owned ones from the copy
		std::vector<dhtt_msgs::msg::Resource> necessary_resources_cp;
		auto owned_resources = container->get_owned_resources();

		// lambda searches first by name then by type for existing resources in the owned list
		auto populate_only_unowned = [&cur_resource, &find_resource_by_type, &find_resource_by_name]
									(std::vector<dhtt_msgs::msg::Resource>& to_populate, std::vector<dhtt_msgs::msg::Resource>& needed, std::vector<dhtt_msgs::msg::Resource>& owned)
		{
			for ( auto resource : needed )
			{
				auto find_func = ( not strcmp("", resource.name.c_str() ) ) ? find_resource_by_type : find_resource_by_name;
				cur_resource = resource;

				auto found = std::find_if(owned.begin(), owned.end(), find_func);

				if ( found == owned.end() )
					to_populate.push_back( resource );
			}
		};

		// make sure to get the necessary and necessary_world lists
		populate_only_unowned(necessary_resources_cp, this->necessary_resources, owned_resources);
		populate_only_unowned(necessary_resources_cp, this->necessary_world_resources, owned_resources);

		// to_ret->activation_potential = this->get_perceived_efficiency(container);
		to_ret->requested_resources = necessary_resources_cp;
		to_ret->owned_resources = owned_resources;
		to_ret->done = this->is_done();
		to_ret->possible = container->is_request_possible(to_ret->requested_resources);
		
		return to_ret;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> ActionType::work_callback( dhtt::Node* container )
	{
		// not sure what goes here for now, work will be called from Node
		(void) container;
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		DHTT_LOG_INFO(this->com_agg, "\tPerforming work");

		// clear the necessary_world_resources in case do_work needs some after execution
		this->necessary_world_resources.clear();

		this->do_work(container);

		to_ret->done = this->is_done();

		// enable the behavior to acquire world resources after execution
		to_ret->requested_resources = this->necessary_world_resources;
		to_ret->added_resources = this->get_added_resources();
		to_ret->removed_resources = this->get_removed_resources();
		to_ret->passed_resources = this->get_retained_resources(container);
		to_ret->released_resources = this->get_released_resources(container);

		to_ret->last_behavior = container->get_node_name();
		to_ret->success = this->is_done();

		return to_ret;
	}

	std::vector<dhtt_msgs::msg::Resource> ActionType::get_released_resources(dhtt::Node* container)
	{
		auto owned = container->get_owned_resources();
		auto retained = this->get_retained_resources(container);

		std::vector<dhtt_msgs::msg::Resource> to_ret;

		dhtt_msgs::msg::Resource to_check;
		auto same_name = [&to_check] ( dhtt_msgs::msg::Resource iter ) { return not strcmp(iter.name.c_str(), to_check.name.c_str()); };

		for ( auto iter : owned )
		{
			to_check = iter;

			auto found = std::find_if(retained.begin(), retained.end(), same_name);
			if ( found == retained.end() )
				to_ret.push_back(iter);
		}

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