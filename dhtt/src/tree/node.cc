#include "dhtt/tree/node.hpp"

namespace dhtt
{
	Node::Node(std::string name, std::string type, std::vector<std::string> params, std::string p_name) : rclcpp::Node( name ), node_type_loader("dhtt", "dhtt::NodeType"), name(name), parent_name(p_name), priority(1)
	{
		this->error_msg = "";
		this->successful_load = true;

		try
		{
			this->logic = node_type_loader.createSharedInstance(type);
		}
		catch (pluginlib::PluginlibException& ex)
		{
			this->error_msg = "Error when loading plugin " + type + ": " + ex.what();
			this->successful_load = false;

			return;
		}

		this->plugin_name = type;

		this->logic->initialize(params);

		// set up services and action servers
		std::string my_register_topic = std::string(TREE_PREFIX) + name + REGISTER_CHILD_POSTFIX;
		std::string my_activation_topic = std::string(TREE_PREFIX) + name + ACTIVATION_POSTFIX;

		this->register_server = this->create_service<dhtt_msgs::srv::InternalServiceRegistration>(my_register_topic, std::bind(&Node::register_child_callback, this, std::placeholders::_1, std::placeholders::_2));

		this->status_pub = this->create_publisher<dhtt_msgs::msg::Node>("/status", 10);

		this->activation_server = rclcpp_action::create_server<dhtt_msgs::action::Activation>(this, my_activation_topic,
			std::bind(&Node::goal_activation_callback, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&Node::cancel_activation_callback, this, std::placeholders::_1),
			std::bind(&Node::activation_accepted_callback, this, std::placeholders::_1));

		this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);
	}

	Node::~Node()
	{
		// empty for now
	}

	bool Node::loaded_successfully()
	{
		return this->successful_load;
	}

	std::string Node::get_error_msg()
	{
		return this->error_msg;
	}

	std::vector<dhtt_msgs::msg::Resource> Node::get_owned_resources()
	{
		return this->owned_resources;
	}

	void Node::set_owned_resources(std::vector<dhtt_msgs::msg::Resource> set_to)
	{
		// this is an unsafe method for now. USE AT YOUR OWN DISCRETION
		this->owned_resources.clear();

		this->owned_resources = set_to;
	}

	void Node::register_with_parent()
	{
		// should only evaluate to false in the case of ROOT
		if ( strcmp(this->parent_name.c_str(), "NONE") )
		{
			std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> req = std::make_shared<dhtt_msgs::srv::InternalServiceRegistration::Request>();
			req->node_name = this->name;

			std::string parent_register_topic = std::string(TREE_PREFIX) + this->parent_name + REGISTER_CHILD_POSTFIX;

			rclcpp::Client<dhtt_msgs::srv::InternalServiceRegistration>::SharedPtr client = this->create_client<dhtt_msgs::srv::InternalServiceRegistration>(parent_register_topic);

			// utilizing the scope herre to avoid namespacing issues
			{
				using namespace std::chrono_literals;
				while ( not client->wait_for_service(1s) and rclcpp::ok() );

				if (rclcpp::ok())
				{

					auto result = client->async_send_request(req);

					if ( result.wait_for(2s) == std::future_status::ready)
					{

						if (not result.get()->success)
						{
							this->error_msg = "Failed to register as a child of parent " + parent_name + ". Is it allowed to have children?";
							this->successful_load = false;
						}
					}
					else
					{
						this->error_msg = "Timed out waiting for response from service " + parent_register_topic + ". Returning in error.";
						this->successful_load = false;
					}
				}
				else
				{
					this->error_msg = "Exited while waiting for service " + parent_register_topic + " to start. Returning in error";
					this->successful_load = false;
				}
			}
		}
	}

	bool Node::remove_child(std::string child_name)
	{
		auto found_name = [&]( std::string to_check ) { return not strcmp(to_check.c_str(), child_name.c_str()); };

		auto found_iter = std::find_if(this->child_names.begin(), this->child_names.end(), found_name);
		int found_index = std::distance(this->child_names.begin(), found_iter);

		this->activation_clients.erase(std::next(this->activation_clients.begin(), found_index));
		this->child_names.erase(found_iter);

		return true;
	}
	
	void Node::async_activate_child(std::string child_name, dhtt_msgs::action::Activation::Goal activation_goal)
	{
		auto found_name = [&]( std::string to_check ) { return not strcmp(to_check.c_str(), child_name.c_str()); };

		auto found_iter = std::find_if(this->child_names.begin(), this->child_names.end(), found_name);
		int found_index = std::distance(this->child_names.begin(), found_iter);

		auto client_iter = std::next(this->activation_clients.begin(), found_index);

		if ( not (*client_iter)->wait_for_action_server() )
		{
			this->error_msg = "Cannot reach action server of requested child, are you sure it started up?";
			return;
		}

		this->expected_responses++;

		auto send_goal_options = rclcpp_action::Client<dhtt_msgs::action::Activation>::SendGoalOptions();

		send_goal_options.result_callback = std::bind(&Node::store_result_callback, this, std::placeholders::_1, child_name);

		(*client_iter)->async_send_goal(activation_goal, send_goal_options);
	}

	bool Node::block_for_responses_from_children()
	{
		while (this->stored_responses < this->expected_responses);

		return true;
	}

	std::map<std::string, dhtt_msgs::action::Activation::Result::SharedPtr> Node::get_activation_results()
	{
		return this->responses;
	}

	std::vector<std::string> Node::get_child_names()
	{
		return this->child_names;
	}

	std::string Node::get_active_child_name()
	{
		return this->active_child_name;
	}

	void Node::update_status( int8_t n_state )
	{
		// update internally
		this->status.state = n_state;

		// send update to status topic
		dhtt_msgs::msg::Node full_status;

		full_status.node_name = this->name;
		full_status.parent = -1;
		full_status.parent_name = this->parent_name;

		full_status.child_name = this->child_names;
		full_status.params = this->logic->params;

		full_status.plugin_name = this->plugin_name;

		full_status.node_status = this->status;

		this->status_pub->publish(full_status);
	}

	rclcpp_action::GoalResponse Node::goal_activation_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const dhtt_msgs::action::Activation::Goal> goal)
	{
		RCLCPP_DEBUG(this->get_logger(), "Received activation from parent.");
		(void) uuid;
		(void) goal;

		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse Node::cancel_activation_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle)
	{
		RCLCPP_DEBUG(this->get_logger(), "Received request to cancel activation.");
		(void) goal_handle;

		// there is definitely room for some logic to cancel the work thread from here

		return rclcpp_action::CancelResponse::ACCEPT;
	}
	
	void Node::activation_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle)
	{
		// make sure the work thread is deconstructed before we make a new one
		this->work_thread.reset();
		this->stored_responses = 0;
		this->expected_responses = 0;
		
		if ( this->status.state == dhtt_msgs::msg::NodeStatus::WAITING or this->status.state == dhtt_msgs::msg::NodeStatus::DONE )
		{
			this->update_status(dhtt_msgs::msg::NodeStatus::ACTIVE);

			// spin up the auction activation thread
			work_thread = std::make_shared<std::thread>(&Node::activate, this, goal_handle);
		}
		else if ( this->status.state == dhtt_msgs::msg::NodeStatus::ACTIVE )
		{
			if ( goal_handle->get_goal()->success )
			{
				this->update_status(dhtt_msgs::msg::NodeStatus::WORKING);

				// spin up the thread for working
				work_thread = std::make_shared<std::thread>(&Node::activate, this, goal_handle);
			}
			else
			{
				this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

				this->propogate_failure_down();
			}
		}
	}

	void Node::store_result_callback( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Activation>::WrappedResult & result, std::string node_name )
	{
		this->stored_responses++;

		this->responses[node_name] = result.result;
	}

	// simpler than it could be especially with some work in interruption handling
	void Node::activate(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle)
	{
		// collect result from children or self
		dhtt_msgs::action::Activation::Result::SharedPtr to_ret;

		if ( this->status.state == dhtt_msgs::msg::NodeStatus::ACTIVE )
		{
			// first deal with any passed resources
			for ( dhtt_msgs::msg::Resource passed_resource : goal_handle->get_goal()->passed_resources )
				this->owned_resources.push_back( passed_resource );

			// then start auction work
			to_ret = this->logic->auction_callback(this);

			this->active_child_name = to_ret->local_best_node;
		}
		else if ( this->status.state == dhtt_msgs::msg::NodeStatus::WORKING )
		{
			// add granted resources to the owned resources for the logic
			for ( dhtt_msgs::msg::Resource granted_resource : goal_handle->get_goal()->granted_resources )
				this->owned_resources.push_back( granted_resource );

			// start work 
			to_ret = this->logic->work_callback(this, goal_handle->get_goal()->success);

			// take passed resources and again add them to owned resources. Remove any released resources
			this->owned_resources.clear();

			for ( dhtt_msgs::msg::Resource passed_resource : to_ret->passed_resources )
				this->owned_resources.push_back( passed_resource );

			if (this->logic->isDone())
				this->update_status(dhtt_msgs::msg::NodeStatus::DONE);
			else
				this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);
		}
		else if ( this->status.state == dhtt_msgs::msg::NodeStatus::DONE )
		{
			to_ret->done = true;
		}

		to_ret->activation_potential = this->calculate_activation_potential();

		// then send result back to parent
		goal_handle->succeed(to_ret);
	}

	void Node::register_child_callback(std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Response> response)
	{
		if (not this->logic->can_add_child())
		{
			response->success = false;

			return;
		}

		// make a client and push back
		rclcpp_action::Client<dhtt_msgs::action::Activation>::SharedPtr n_client = rclcpp_action::create_client<dhtt_msgs::action::Activation>(this, std::string(TREE_PREFIX) + request->node_name + ACTIVATION_POSTFIX);

		this->child_names.push_back(request->node_name);
		this->activation_clients.push_back(n_client);
		response->success = true;
	}

	// preconditions are stored as key: val pairs in a vector of strings in logic. Here we need to check them against world information
	bool Node::check_preconditions()
	{
		return true;
	}

	double Node::calculate_activation_potential()
	{
		return this->logic->get_perceived_efficiency() * (this->priority + (int) this->owned_resources.size());
	}

	void Node::propogate_failure_down()
	{
		dhtt_msgs::action::Activation::Goal n_goal;

		n_goal.success = false;

		this->async_activate_child(this->active_child_name, n_goal);

		this->block_for_responses_from_children();

		this->active_child_name = "";
	}
}