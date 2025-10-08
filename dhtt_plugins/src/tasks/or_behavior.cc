#include "dhtt_plugins/tasks/or_behavior.hpp"

namespace dhtt_plugins
{
	void OrBehavior::initialize(std::vector<std::string> params) 
	{
		(void) params;

		this->has_chosen_child = false;
		this->child_has_run = false;
		this->child_done = false;

		this->activation_potential = 0;
		this->activated_child_name = "";

		this->preconditions.logical_operator = dhtt_utils::LOGICAL_OR;
		this->postconditions.logical_operator = dhtt_utils::LOGICAL_OR;

		this->parse_params(params);

		this->children_allowed = true;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> OrBehavior::auction_callback( dhtt::Node* container ) 
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		std::vector<std::string> children = container->get_child_names();

		DHTT_LOG_INFO(this->com_agg, "\tAuction callback started activating children...");

		if ( (int) children.size() == 0 )
		{
			to_ret->done = true;

			container->update_status(dhtt_msgs::msg::NodeStatus::DONE);

			return to_ret;
		}

		// if we get to this point and the child has not run we can reselect because it did not succeed further up the tree
		if ( not this->child_has_run )
			this->has_chosen_child = false;

		dhtt_msgs::action::Activation::Result child_req;

		dhtt_msgs::action::Activation::Goal n_goal;
		n_goal.passed_resources = container->get_owned_resources();

		if ( not this->has_chosen_child )
		{
			// activate all children for activation potential calculation and give the first one the resources that we were passed
			container->activate_all_children(n_goal);

			// wait for result
			container->block_for_activation_from_children();

			DHTT_LOG_DEBUG(this->com_agg, "Responses received...");

			auto results = container->get_activation_results();

			// pick highest to activate
			double current_max = -1;

			for ( auto const& x : results )
			{
				if ( x.second.activation_potential > current_max and x.second.possible and not x.second.done )
				{
					current_max = x.second.activation_potential;
					this->activated_child_name = x.first;
				}

				// if we for some reason already have a finished child then we have to pass that up and we are also done
				if ( x.second.done )
				{
					current_max = 1;
					this->activated_child_name = x.first;

					break;
				}
			}

			// if we don't find one we can just pick the first since impossible requests won't cause issues anyway
			if ( current_max == -1 )
			{
				this->activated_child_name = (*results.begin()).first;
			}

			// collect parameters and change state to chosen
			this->has_chosen_child = true;

			child_req = results[this->activated_child_name];

			n_goal.success = false;

			for ( auto iter : container->get_child_names() )
				if ( strcmp( iter.c_str(), this->activated_child_name.c_str() ) and results[iter].possible )
					container->async_activate_child(iter, n_goal);
				
			container->block_for_activation_from_children();
			(void) container->get_activation_results();
		}
		else
		{
			// activate chosen child
			container->async_activate_child(this->activated_child_name, n_goal);

			container->block_for_activation_from_children();

			DHTT_LOG_DEBUG(this->com_agg, "Responses received...");

			auto result = container->get_activation_results();

			// save result for constructing the new one
			child_req = result[this->activated_child_name];
		}

		this->activation_potential = child_req.activation_potential;

		DHTT_LOG_WARN(this->com_agg, "\tRecommending child [" << this->activated_child_name << "] for activation which is done [" << child_req.done << "]...") ;

		to_ret->local_best_node = this->activated_child_name;
		to_ret->requested_resources = child_req.requested_resources;
		// to_ret->owned_resources = child_req.owned_resources;
		to_ret->done = child_req.done;
		to_ret->possible = child_req.possible;

		if ( to_ret->done )
			this->child_done = true;

		return to_ret;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> OrBehavior::work_callback( dhtt::Node* container ) 
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		// send success to winning child		
		dhtt_msgs::action::Activation::Goal n_goal;

		n_goal.success = true;
		n_goal.granted_resources = container->get_owned_resources();

		std::string active = container->get_active_child_name();

		container->async_activate_child(active, n_goal);

		// block until the child is done
		container->block_for_activation_from_children();

		// get result
		auto result = container->get_activation_results()[active];

		// child has now run at least once so we can no longer reselect
		if ( result.success )
			this->child_has_run = true;

		// if they are done then so are we
		if ( result.done )
			this->child_done = true;

		// change hands of resources and pass up
		to_ret->passed_resources = result.passed_resources;
		to_ret->requested_resources = result.requested_resources;
		to_ret->released_resources = result.released_resources;
		to_ret->added_resources = result.added_resources;
		to_ret->removed_resources = result.removed_resources;
		to_ret->last_behavior = result.last_behavior;
		to_ret->done = this->is_done();
		to_ret->success = result.success;

		return to_ret;
	}

	void OrBehavior::maintain_conditions( dhtt::Node* container )
	{
		// this one is pretty simple 
		auto response_cp = container->get_condition_results();

		// append all of the predicates
		this->preconditions.predicates.clear();
		this->preconditions.conjunctions.clear();
		this->postconditions.predicates.clear();
		this->postconditions.conjunctions.clear();

		for ( auto const& child_response : response_cp )
		{
			auto precon_struct = dhtt_utils::convert_to_struct(child_response.second.preconditions);
			auto postcon_struct = dhtt_utils::convert_to_struct(child_response.second.postconditions);

			// first grab the preconditions
			if ( precon_struct.logical_operator == dhtt_utils::LOGICAL_OR or precon_struct.logical_operator == dhtt_utils::LOGICAL_OTHER )
			{
				dhtt_utils::append_predicate_conjunction(this->preconditions, precon_struct);
				dhtt_utils::append_predicate_conjunction(this->postconditions, postcon_struct);
			}
			else
			{
				// here just append it as a conjunction since the alternative is to use the distributive property
				this->preconditions.conjunctions.push_back( dhtt_utils::conjunction_copy( precon_struct ) );
				this->postconditions.conjunctions.push_back( dhtt_utils::conjunction_copy( postcon_struct ) );
			}
		}

		// finally remove any repeated predicates from the predicate lists in pre and postconditions
		dhtt_utils::remove_predicate_duplicates(this->preconditions);
		dhtt_utils::remove_predicate_duplicates(this->postconditions);

		dhtt_utils::flatten_predicates(this->preconditions);
		dhtt_utils::flatten_predicates(this->postconditions);
	}


	void OrBehavior::parse_params( std::vector<std::string> params ) 
	{
		// if ( (int) params.size() > 0 )
		// 	throw std::invalid_argument("Or Behaviors do not take parameters but " + ((int) params.size()) + std::string(" were given. Returning in error."));
	
		(void) params;
	}

	double OrBehavior::get_perceived_efficiency(dhtt::Node* container) 
	{
		(void) container;
		return this->activation_potential;
	}

	bool OrBehavior::is_done() 
	{
		return this->child_done;
	}

}