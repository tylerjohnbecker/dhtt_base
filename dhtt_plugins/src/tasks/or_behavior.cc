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

		this->parse_params(params);

		this->children_allowed = true;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> OrBehavior::auction_callback( dhtt::Node* container ) 
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		std::vector<std::string> children = container->get_child_names();

		RCLCPP_INFO(container->get_logger(), "\tAuction callback started activating children...");

		if ( (int) children.size() == 0 )
		{
			to_ret->done = true;

			container->update_status(dhtt_msgs::msg::NodeStatus::DONE);

			return to_ret;
		}

		// if we get to this point and the child has not run we can reselect because it did not succeed further up the tree
		if ( not this->child_has_run )
			this->has_chosen_child = false;

		std::shared_ptr<dhtt_msgs::action::Activation::Result> child_req;

		dhtt_msgs::action::Activation::Goal n_goal;
		n_goal.passed_resources = container->get_owned_resources();

		if ( not this->has_chosen_child )
		{
			// activate all children for activation potential calculation and give the first one the resources that we were passed
			container->activate_all_children(n_goal);

			// wait for result
			container->block_for_responses_from_children();

			RCLCPP_DEBUG(container->get_logger(), "Responses received...");

			auto results = container->get_activation_results();

			// pick highest to activate
			double current_max = -1;

			for ( auto const& x : results )
			{
				if ( x.second->activation_potential > current_max and x.second->possible )
				{
					current_max = x.second->activation_potential;
					this->activated_child_name = x.first;
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
		}
		else
		{
			// activate chosen child
			container->async_activate_child(this->activated_child_name, n_goal);

			container->block_for_responses_from_children();

			RCLCPP_DEBUG(container->get_logger(), "Responses received...");

			auto result = container->get_activation_results();

			// save result for constructing the new one
			child_req = result[this->activated_child_name];
		}

		this->activation_potential = child_req->activation_potential;

		RCLCPP_WARN(container->get_logger(), "\tRecommending child [%s] for activation..", this->activated_child_name.c_str()) ;

		to_ret->local_best_node = this->activated_child_name;
		to_ret->requested_resources = child_req->requested_resources;
		to_ret->owned_resources = child_req->owned_resources;
		to_ret->done = child_req->done;
		to_ret->possible = child_req->possible;

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
		container->block_for_responses_from_children();

		// get result
		auto result = container->get_activation_results()[active];

		// child has now run at least once so we can no longer reselect
		this->child_has_run = true;

		// if they are done then so are we
		if ( result->done )
			this->child_done = true;

		// change hands of resources and pass up
		to_ret->passed_resources = result->passed_resources;
		to_ret->released_resources = result->released_resources;
		to_ret->done = this->is_done();

		return to_ret;
	}

	void OrBehavior::parse_params( std::vector<std::string> params ) 
	{
		if ( (int) params.size() > 0 )
			throw std::invalid_argument("Or Behaviors do not take parameters but " + ((int) params.size()) + std::string(" were given. Returning in error."));
	
		(void) params;
	}

	double OrBehavior::get_perceived_efficiency() 
	{
		return this->activation_potential;
	}

	bool OrBehavior::is_done() 
	{
		return this->child_done;
	}

}