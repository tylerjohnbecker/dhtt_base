#include "dhtt_plugins/tasks/and_behavior.hpp"

namespace dhtt_plugins
{
	void AndBehavior::initialize(std::vector<std::string> params) 
	{
		this->num_active_children = 0;
		this->activation_potential = 0;

		this->parse_params(params);

		this->children_allowed = true;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> AndBehavior::auction_callback( dhtt::Node* container ) 
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		// activate all children which aren't done yet
		dhtt_msgs::action::Activation::Goal n_goal;
		n_goal.passed_resources = container->get_owned_resources();

		container->activate_all_children(n_goal);

		// wait for responses
		container->block_for_responses_from_children();

		auto results = container->get_activation_results();

		// pick the highest activation potential request which is possible (also calculate is done from here)
		std::vector<std::string> active_children;
		std::string local_best_child = "";

		double current_max_activation_potential = -1;

		int num_children = 0;
		double activation_potential_sum = 0.0;

		for ( auto const& x : results )
		{
			if ( not x.second->done )
			{
				num_children++;
				activation_potential_sum += x.second->activation_potential;

				if ( x.second->activation_potential > current_max_activation_potential and x.second->possible )
				{
					current_max_activation_potential = x.second->activation_potential;

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

		// check if a possible child exists
		to_ret->possible = not strcmp( "", local_best_child.c_str() );

		// send stop back to the rest
		n_goal.success = false;

		for ( std::string active_children_iter : active_children )
			container->async_activate_child(active_children_iter, n_goal);

		container->block_for_responses_from_children();

		// return winner
		to_ret->local_best_node = local_best_child;

		if ( to_ret->possible )
		{
			to_ret->requested_resources = results[local_best_child]->requested_resources;
			to_ret->owned_resources = results[local_best_child]->owned_resources;
		}

		to_ret->done = this->isDone();

		return to_ret;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> AndBehavior::work_callback( dhtt::Node* container ) 
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		// propogate success to winner
		dhtt_msgs::action::Activation::Goal n_goal;
		n_goal.passed_resources = container->get_owned_resources();
		n_goal.success = true;

		std::string active_child = container->get_active_child_name();

		container->async_activate_child(active_child, n_goal);

		// wait for response
		container->block_for_responses_from_children();

		auto result = container->get_activation_results()[active_child];

		this->num_active_children--;

		// copy and return message with this node as the local node
		to_ret->released_resources = result->released_resources;
		to_ret->passed_resources = result->passed_resources;
		to_ret->done = this->isDone();

		return to_ret;
	}

	void AndBehavior::parse_params( std::vector<std::string> params ) 
	{
		if ( (int) params.size() > 0 )
			throw std::invalid_argument("Or Behaviors do not take parameters but " + ((int) params.size()) + std::string(" were given. Returning in error."));
	
		(void) params;
	}

	double AndBehavior::get_perceived_efficiency() 
	{
		return this->activation_potential;
	}

	std::vector<dhtt_msgs::msg::Resource> AndBehavior::get_retained_resources( dhtt::Node* container ) 
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}

	std::vector<dhtt_msgs::msg::Resource> AndBehavior::get_released_resources( dhtt::Node* container ) 
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}

	bool AndBehavior::isDone() 
	{
		return this->num_active_children == 0;
	}
}