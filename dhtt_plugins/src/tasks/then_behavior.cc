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

		this->parse_params(params);

		return;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> ThenBehavior::auction_callback( dhtt::Node* container )
	{
		std::shared_ptr<dhtt_msgs::action::Activation::Result> to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		std::vector<std::string> children = container->get_child_names();

		if ( (int) children.size() == 0 )
		{
			to_ret->done = true;

			container->update_status(dhtt_msgs::msg::NodeStatus::DONE);

			return to_ret;
		}

		if ( not this->started_activation )
			this->child_queue_index = 0;
		
		this->child_queue_size = (int) children.size();

		bool first = true;

		dhtt_msgs::action::Activation::Goal n_goal;

		n_goal.passed_resources = container->get_owned_resources();

		// activate all children for activation potential calculation and give the first one the resources that we were passed
		for (std::vector<std::string>::iterator name_iter = std::next(children.begin(), this->child_queue_index) ; name_iter != children.end() ; name_iter++)
		{
			container->async_activate_child(*name_iter, n_goal);
		
			if (first)
			{
				n_goal.passed_resources.clear();

				first = false;
			}
		}

		container->block_for_responses_from_children();

		auto results = container->get_activation_results();

		std::string first_child_in_queue = children.begin()[this->child_queue_index];

		to_ret->local_best_node = first_child_in_queue;

		// calculate activation potential
		double total_sum = 0;
		int total_num_children = (int) results.size();

		for (auto const& x  : results)
		{
			total_sum += x.second->activation_potential;

			if ( not strcmp( x.second->local_best_node.c_str(), first_child_in_queue.c_str() ) )
			{
				// add front of queue to result
				to_ret->done = x.second->done;
				to_ret->requested_resources = x.second->requested_resources;
				to_ret->owned_resources = x.second->owned_resources;
			}
		}

		this->activation_potential = total_sum / total_num_children;

		// send failure back to the children not at the front of the queue
		n_goal.success = false;

		for (std::vector<std::string>::iterator name_iter = std::next(children.begin(), this->child_queue_index + 1) ; name_iter != children.end() ; name_iter++)
			container->async_activate_child(*name_iter, n_goal);
		
		// return the result
		return to_ret;
	}

	std::shared_ptr<dhtt_msgs::action::Activation::Result> ThenBehavior::work_callback( dhtt::Node* container, bool success)
	{
		// not sure what goes here for now, work will be called from Node
		(void) container;
		(void) success;

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

		// increment queue if the child is done
		if ( result->done )
			this->child_queue_index++;

		// change hands of resources and pass up
		if ( not this->isDone() )
			container->set_owned_resources(result->passed_resources);
		else
			to_ret->passed_resources = result->passed_resources;

		to_ret->released_resources = result->released_resources;
		to_ret->done = this->isDone();

		return to_ret;
	}

	void ThenBehavior::parse_params( std::vector<std::string> params )
	{
		if ( (int) params.size() > 0 )
			throw std::invalid_argument("Too many parameters passed to node. Only activation potential required.");

		if ( (int) params.size() == 0 )
		{
			this->activation_potential = 0;

			return;
		}

	}

	double ThenBehavior::get_perceived_efficiency()
	{
		// just give random perceived efficiency
		return this->activation_potential;
	}

	std::vector<dhtt_msgs::msg::Resource> ThenBehavior::get_retained_resources( dhtt::Node* container )
	{
		(void) container;

		return std::vector<dhtt_msgs::msg::Resource>();
	}

	std::vector<dhtt_msgs::msg::Resource> ThenBehavior::get_released_resources( dhtt::Node* container )
	{
		(void) container;

		return container->get_owned_resources();
	}

	bool ThenBehavior::isDone()
	{
		return this->child_queue_index >= this->child_queue_size;
	}
}