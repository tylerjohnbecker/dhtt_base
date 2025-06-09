#include "dhtt_plugins/tasks/then_behavior.hpp"


namespace dhtt_plugins
{

	void ThenBehavior::initialize(std::vector<std::string> params)
	{
		// these just supress warnings regarding not using function params
		(void) params;

		// only allowed for now until the task logic is implemented
		this->children_allowed = true;

		this->child_queue_index = -1;
		this->child_queue_size = -1;
		this->started_activation = false;

		this->activation_potential = 0;

		this->created = false;

		this->preconditions.logical_operator = dhtt_utils::LOGICAL_AND;
		this->postconditions.logical_operator = dhtt_utils::LOGICAL_AND;

		this->parse_params(params);

		this->created = true;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> ThenBehavior::auction_callback( dhtt::Node* container )
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		std::vector<std::string> children = container->get_child_names();

		RCLCPP_INFO(container->get_logger(), "\tAuction callback started activating children...");

		if ( not this->started_activation )
		{
			this->child_queue_index = 0;
			this->started_activation = true;
		}

		this->child_queue_size = (int) children.size();

		if ( (int) children.size() == 0 or this->child_queue_index >= this->child_queue_size ) 
		{
			to_ret->done = true;

			container->update_status(dhtt_msgs::msg::NodeStatus::DONE);

			return to_ret;
		}

		dhtt_msgs::action::Activation::Goal n_goal;

		n_goal.passed_resources = container->get_passed_resources();

		for (auto resource: container->get_owned_resources())
			n_goal.passed_resources.push_back(resource);

		auto results = container->get_activation_results();
		std::string first_child_in_queue;

		for ( this->next = 0 ; ( this->child_queue_index + this->next ) < this->child_queue_size ; this->next++ )
		{
			first_child_in_queue = *(children.begin() + this->child_queue_index + this->next);

			// for (auto resource : n_goal.passed_resources)
			// 	RCLCPP_FATAL(container->get_logger(), "type: %d, owned: %d", resource.type, resource.locked);

			container->async_activate_child(first_child_in_queue, n_goal);

			container->block_for_activation_from_children();

			results = container->get_activation_results();

			if (not results[first_child_in_queue].done)
				break;
		}

		RCLCPP_INFO(container->get_logger(), "Responses received...");

		// while ( results[first_child_in_queue]->done and this->child_queue_index < this->child_queue_size - 1 )
		// {
		// 	this->child_queue_index++;
		// 	first_child_in_queue = children[this->child_queue_index];
		// }

		to_ret->local_best_node = first_child_in_queue;

		// calculate activation potential
		// double total_sum = 0;
		// int total_num_children = (int) results.size();
		//
		// for (auto const& x  : results)
		// 	total_sum += x.second->activation_potential;

		RCLCPP_WARN(container->get_logger(), "\tRecommending child [%s] for activation in queue position %d..", first_child_in_queue.c_str(), this->child_queue_index) ;

		to_ret->requested_resources = results[first_child_in_queue].requested_resources;
		to_ret->owned_resources = results[first_child_in_queue].owned_resources;
		to_ret->done = results[first_child_in_queue].done;
		to_ret->possible = results[first_child_in_queue].possible;

		this->activation_potential = results[first_child_in_queue].activation_potential;// / total_num_children;

		// make sure to send back failure to all if nothing is possible
		// if ( not to_ret->possible )
		// {
		// 	n_goal.success = false;
		//
		// 	container->async_activate_child(first_child_in_queue, n_goal);
		// 	container->block_for_responses_from_children();
		// }

		// for (std::vector<std::string>::iterator name_iter = children.begin() + next ; name_iter != children.end() ; name_iter++)
		// 	if (results[*name_iter]->possible)
		// 		container->async_activate_child(*name_iter, n_goal);
		//
		// container->block_for_responses_from_children();
		
		// return the result
		return to_ret;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> ThenBehavior::work_callback( dhtt::Node* container )
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		std::lock_guard<std::mutex> guard(this->queue_index_mut);

		this->child_queue_index += this->next;

		// send success to winning child		
		dhtt_msgs::action::Activation::Goal n_goal;

		n_goal.success = true;
		n_goal.granted_resources = container->get_owned_resources();

		for (auto resource : container->get_passed_resources())
			n_goal.granted_resources.push_back(resource);

		std::string active = container->get_active_child_name();

		RCLCPP_WARN(container->get_logger(), "Telling child %s to work!", active.c_str());

		container->async_activate_child(active, n_goal);

		// block until the child is done
		container->block_for_activation_from_children();

		// get result
		auto result = container->get_activation_results()[active];

		// increment queue if the child is done
		if ( result.done )
		{
			// RCLCPP_INFO(container->get_logger(), "Child done incrementing queue index!");

			this->child_queue_index++;
		}

		RCLCPP_WARN(container->get_logger(), "Child finished running, %d left!", (this->child_queue_size - this->child_queue_index) );

		// change hands of resources and pass up
		if ( not this->is_done() )
		{
			container->set_passed_resources(result.passed_resources);
		}
		else
		{
			to_ret->passed_resources = result.passed_resources;
			container->set_passed_resources(std::vector<dhtt_msgs::msg::Resource>());
		}

		// RCLCPP_INFO(container->get_logger(), "Queue index %d and Queue size %d and is done %d", this->child_queue_index, this->child_queue_size, this->is_done());

		to_ret->released_resources = result.released_resources;
		to_ret->last_behavior = result.last_behavior;
		to_ret->done = this->is_done();
		to_ret->success = result.success;

		return to_ret;
	}

	void ThenBehavior::maintain_conditions( dhtt::Node* container )
	{
		// this one is pretty simple 
		auto response_cp = container->get_condition_results();

		// append all of the predicates
		this->preconditions.predicates.clear();
		this->preconditions.conjunctions.clear();
		this->postconditions.predicates.clear();
		this->postconditions.conjunctions.clear();

		bool first = true;

		std::vector<dhtt_utils::PredicateConjunction> prev_postconditions;

		// loop through strictly in the order of the children list from the container and generate the preconditions list
		for ( auto const& child_name : container->get_child_names() )
		{
			auto inc_preconditions = dhtt_utils::convert_to_struct(response_cp[child_name].preconditions);
			auto inc_postconditions = dhtt_utils::convert_to_struct(response_cp[child_name].postconditions);
			
			dhtt_utils::PredicateConjunction adjusted_preconditions;

			std::shared_ptr<dhtt_utils::PredicateConjunction> tmp = std::make_shared<dhtt_utils::PredicateConjunction>();
			*tmp = inc_postconditions;

			if ( first )
			{
				prev_postconditions.push_back(inc_postconditions);
				
				adjusted_preconditions = inc_preconditions;
				
				first = false;
			}
			else
			{
				adjusted_preconditions = dhtt_utils::remove_partial_dependencies(prev_postconditions, inc_preconditions);

				prev_postconditions.push_back(inc_postconditions);
			}

			dhtt_utils::append_predicate_conjunction(this->preconditions, adjusted_preconditions);

			// Now copy over the postconditions
			if ( inc_preconditions.logical_operator == dhtt_utils::LOGICAL_AND or inc_preconditions.logical_operator == dhtt_utils::LOGICAL_OTHER )
				dhtt_utils::append_predicate_conjunction(this->postconditions, inc_postconditions);
			else
				this->postconditions.conjunctions.push_back( tmp );
			// RCLCPP_WARN(container->get_logger(), "%s", dhtt_utils::to_string(this->postconditions).c_str());
		}

		// finally remove any repeated predicates from the predicate lists in pre and postconditions
		dhtt_utils::remove_predicate_duplicates(this->preconditions);
		dhtt_utils::remove_predicate_partial_duplicates(this->postconditions);

		dhtt_utils::flatten_predicates(this->preconditions);
		dhtt_utils::flatten_predicates(this->postconditions);

		// RCLCPP_ERROR(container->get_logger(), "%s", dhtt_utils::to_string(this->preconditions).c_str());
		// RCLCPP_ERROR(container->get_logger(), "%s", dhtt_utils::to_string(this->postconditions).c_str());
	}


	void ThenBehavior::parse_params( std::vector<std::string> params )
	{
		// if ( (int) params.size() > 0 )
		// 	throw std::invalid_argument("Then Behaviors do not take parameters but " + ((int) params.size()) + std::string(" were given. Returning in error."));

		// if ( (int) params.size() == 0 )
		// {
		// 	this->activation_potential = 0;

		// 	return;
		// }

		// parse reorderings after created

		(void) params;

		if (this->created)
		{
			std::lock_guard<std::mutex> guard(this->queue_index_mut);
		
			if ( (int) params.size() == 0 )
				throw std::invalid_argument("Need at least one parameter to change, but no params were given...");

			auto colon_index = params[0].find(':');
			std::string key = params[0].substr(0, colon_index);
			std::string val = params[0].substr(colon_index + 2, params[0].size() - colon_index);

			if ( strcmp(key.c_str(), "child_queue_index") )
				throw std::invalid_argument("Invalid parameter passed: " + key + " expected \"child_queue_index: int\"");

			this->child_queue_index = atoi(val.c_str());

			if ( this->child_queue_index < 0 or this->child_queue_index > this->child_queue_size )
				throw std::invalid_argument("Invalid queue index passed " + std::to_string(this->child_queue_index) + "... Returning in error!");

			// throw std::invalid_argument(std::to_string(this->child_queue_index));
		}
	}

	double ThenBehavior::get_perceived_efficiency()
	{
		// just give random perceived efficiency
		return this->activation_potential;
	}

	bool ThenBehavior::is_done()
	{
		return this->child_queue_index >= this->child_queue_size;
	}
}