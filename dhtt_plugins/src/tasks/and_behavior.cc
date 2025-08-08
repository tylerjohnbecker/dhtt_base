#include "dhtt_plugins/tasks/and_behavior.hpp"

namespace dhtt_plugins
{
	void AndBehavior::initialize(std::vector<std::string> params) 
	{
		this->num_active_children = 0;
		this->activation_potential = 0;

		this->preconditions.logical_operator = dhtt_utils::LOGICAL_AND;
		this->postconditions.logical_operator = dhtt_utils::LOGICAL_AND;

		this->parse_params(params);

		this->children_allowed = true;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> AndBehavior::auction_callback( dhtt::Node* container ) 
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		// activate all children which aren't done yet
		dhtt_msgs::action::Activation::Goal n_goal;
		n_goal.passed_resources = container->get_owned_resources();

		DHTT_LOG_INFO(this->com_agg, "\tAuction callback started activating children...");

		container->activate_all_children(n_goal);

		// wait for responses
		container->block_for_activation_from_children();

		DHTT_LOG_DEBUG(this->com_agg, "\tResponses received...");

		auto results = container->get_activation_results();

		// pick the highest activation potential request which is possible (also calculate is done from here)
		std::vector<std::string> active_children;
		std::string local_best_child = "";

		double current_max_activation_potential = -1;

		int num_children = 0;
		double activation_potential_sum = 0.0;

		for ( auto const& x : results )
		{
			if ( not x.second.done )
			{
				DHTT_LOG_DEBUG(this->com_agg, "Evaluating child [" << x.first << "] with activation potential " << x.second.activation_potential << "...");
				num_children++;
				activation_potential_sum += x.second.activation_potential;

				if ( x.second.activation_potential > current_max_activation_potential and x.second.possible )
				{
					current_max_activation_potential = x.second.activation_potential;

					// this wasn't pushed onto active children the first time so do it now
					active_children.push_back(x.first);

					local_best_child = x.first;
				}
				else
				{
					active_children.push_back(x.first);
				}
			}
		}

		this->num_active_children = num_children;
		this->activation_potential = activation_potential_sum / num_children;
		to_ret->activation_potential = activation_potential;

		// check if a possible child exists
		if ( strcmp( "", local_best_child.c_str() ) )
		{
			to_ret->possible = results[local_best_child].possible;
		}

		// send stop back to the rest
		n_goal.success = false;

		for ( std::string active_children_iter : active_children )
			if (strcmp(active_children_iter.c_str(), local_best_child.c_str()) and results[active_children_iter].possible )
				container->async_activate_child(active_children_iter, n_goal);
	
		container->block_for_activation_from_children();
		container->get_activation_results();

		// return winner
		to_ret->local_best_node = local_best_child;

		DHTT_LOG_WARN(this->com_agg, "\tRecommending child [" << local_best_child << "] for activation with potential " << results[local_best_child].activation_potential << "...");

		if ( to_ret->possible )
		{
			to_ret->requested_resources = results[local_best_child].requested_resources;
			to_ret->owned_resources = results[local_best_child].owned_resources;
		}

		to_ret->done = this->is_done();

		return to_ret;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> AndBehavior::work_callback( dhtt::Node* container ) 
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		// propogate success to winner
		dhtt_msgs::action::Activation::Goal n_goal;
		n_goal.granted_resources = container->get_owned_resources();
		n_goal.success = true;

		std::string active_child = container->get_active_child_name();

		container->async_activate_child(active_child, n_goal);

		// wait for response
		container->block_for_activation_from_children();

		auto result = container->get_activation_results()[active_child];

		if (result.done)
			this->num_active_children--;

		DHTT_LOG_INFO(this->com_agg, "Child finished running, " << this->num_active_children << " active children left.");

		// copy and return message with this node as the local node
		to_ret->released_resources = result.released_resources;
		to_ret->released_resources.insert(to_ret->released_resources.end(),  result.passed_resources.begin(), result.passed_resources.end());
		// to_ret->passed_resources = result.passed_resources;
		to_ret->last_behavior = result.last_behavior;
		to_ret->done = this->is_done();
		to_ret->success = result.success;

		return to_ret;
	}

	void AndBehavior::maintain_conditions( dhtt::Node* container )
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
			if ( precon_struct.logical_operator == dhtt_utils::LOGICAL_AND or precon_struct.logical_operator == dhtt_utils::LOGICAL_OTHER )
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
		dhtt_utils::remove_predicate_partial_duplicates(this->preconditions);
		dhtt_utils::remove_predicate_partial_duplicates(this->postconditions);
		
		dhtt_utils::flatten_predicates(this->preconditions);
		dhtt_utils::flatten_predicates(this->postconditions);
	}

	void AndBehavior::parse_params( std::vector<std::string> params ) 
	{
		// if ( (int) params.size() > 0 )
		// 	throw std::invalid_argument("And Behaviors do not take parameters but " + ((int) params.size()) + std::string(" were given. Returning in error."));
	
		(void) params;
	}

	double AndBehavior::get_perceived_efficiency(dhtt::Node* container) 
	{
		(void) container;
		return this->activation_potential;
	}

	bool AndBehavior::is_done() 
	{
		return this->num_active_children == 0;
	}
}