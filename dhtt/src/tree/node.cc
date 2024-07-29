#include "dhtt/tree/node.hpp"

namespace dhtt
{
	Node::Node(std::string name, std::string type, std::vector<std::string> params, std::string p_name) : rclcpp::Node( name ), node_type_loader("dhtt", "dhtt::NodeType"), 
			name(name), parent_name(p_name), priority(1), resource_status_updated(false)
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

	std::vector<dhtt_msgs::msg::Resource> Node::get_passed_resources()
	{
		return this->passed_resources;
	}

	std::vector<dhtt_msgs::msg::Resource> Node::get_resource_state()
	{
		return this->available_resources;
	}

	void Node::set_passed_resources(std::vector<dhtt_msgs::msg::Resource> set_to)
	{
		// this is an unsafe method for now. USE AT YOUR OWN DISCRETION
		this->passed_resources.clear();

		this->passed_resources = set_to;
	}

	void Node::register_with_parent()
	{
		// should only evaluate to false in the case of ROOT
		if ( strcmp(this->parent_name.c_str(), "NONE") )
		{
			std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> req = std::make_shared<dhtt_msgs::srv::InternalServiceRegistration::Request>();
			req->node_name = this->name;

			std::string parent_register_topic = std::string(TREE_PREFIX) + "/" + this->parent_name + REGISTER_CHILD_POSTFIX;

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

	void Node::register_servers()
	{
		// set up services and action servers
		std::string my_register_topic = std::string(TREE_PREFIX) + "/" + name + REGISTER_CHILD_POSTFIX;
		std::string my_activation_topic = std::string(TREE_PREFIX) + "/" + name + ACTIVATION_POSTFIX;
		std::string resources_topic = std::string(TREE_PREFIX) + RESOURCES_POSTFIX;

		this->register_server = this->create_service<dhtt_msgs::srv::InternalServiceRegistration>(my_register_topic, std::bind(&Node::register_child_callback, this, std::placeholders::_1, std::placeholders::_2));

		this->status_pub = this->create_publisher<dhtt_msgs::msg::Node>("/status", 10);

		this->resources_sub = this->create_subscription<dhtt_msgs::msg::Resources>(resources_topic, 10, std::bind(&Node::resource_availability_callback, this, std::placeholders::_1));

		this->activation_server = rclcpp_action::create_server<dhtt_msgs::action::Activation>(this, my_activation_topic,
			std::bind(&Node::goal_activation_callback, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&Node::cancel_activation_callback, this, std::placeholders::_1),
			std::bind(&Node::activation_accepted_callback, this, std::placeholders::_1));

		this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

		RCLCPP_INFO(this->get_logger(), "\tServers registered for node %s", this->name.c_str());
	}

	bool Node::remove_child(std::string child_name)
	{
		auto found_name = [&]( std::string to_check ) { return not strcmp(to_check.c_str(), child_name.c_str()); };

		auto found_iter = std::find_if(this->child_names.begin(), this->child_names.end(), found_name);

		if ( found_iter == this->child_names.end() )
			return false;

		int found_index = std::distance(this->child_names.begin(), found_iter);

		this->activation_clients.erase(std::next(this->activation_clients.begin(), found_index));
		this->child_names.erase(found_iter);

		// it is possible for the child to have sent an activation response right before we remove it
		if (this->responses.find(child_name) != this->responses.end())
		{
			RCLCPP_DEBUG(this->get_logger(), "Removing activation response from child to be deleted");
			this->responses.erase(this->responses.find(child_name));
		}

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

	void Node::activate_all_children(dhtt_msgs::action::Activation::Goal activation_goal)
	{
		for (std::string iter : this->child_names)
			this->async_activate_child(iter, activation_goal);
	}

	bool Node::block_for_responses_from_children()
	{
		// RCLCPP_INFO(this->get_logger(), "Waiting for %d responses from children, currently received %d...", this->expected_responses, this->stored_responses );

		while (this->stored_responses < this->expected_responses);

		// RCLCPP_INFO(this->get_logger(), "Responses received!");

		return true;
	}

	std::map<std::string, dhtt_msgs::action::Activation::Result::SharedPtr> Node::get_activation_results()
	{	
		// reset values for storing responses
		this->stored_responses = 0;
		this->expected_responses = 0;

		// RCLCPP_INFO(this->get_logger(), "Responses requested so deleting local values...");

		std::map<std::string, dhtt_msgs::action::Activation::Result::SharedPtr> to_ret;

		for ( auto const& x : this->responses )
			to_ret[x.first] = x.second;

		this->responses.clear();

		return to_ret;
	}

	std::vector<std::string> Node::get_child_names()
	{
		return this->child_names;
	}

	std::string Node::get_active_child_name()
	{
		return this->active_child_name;
	}

	std::string Node::get_node_name()
	{
		return this->name;
	}

	bool Node::is_request_possible(std::vector<dhtt_msgs::msg::Resource> requested_resources)
	{
		bool is_possible = true;
	
		dhtt_msgs::msg::Resource looking_for;
		auto find_fulfill = [&]( dhtt_msgs::msg::Resource to_check )
		{
			return (looking_for.type == to_check.type) and not to_check.locked;
		};

		for ( dhtt_msgs::msg::Resource check : requested_resources )
		{
			looking_for = check;

			std::vector<dhtt_msgs::msg::Resource>::iterator found = std::find_if(this->available_resources.begin(), this->available_resources.end(), find_fulfill);

			while ( found != this->available_resources.end() )
			{
				if ((*found).locked == false )
					break;
				else
					found = std::find_if(found, this->available_resources.end(), find_fulfill);
			}

			if ( found == this->available_resources.end() )
			{
				is_possible = false;
				break;
			}
		}

		return is_possible;
	}

	void Node::update_status( int8_t n_state )
	{
		// update internally
		this->status.state = n_state;

		// send update to status topic
		dhtt_msgs::msg::Node full_status;

		full_status.head.stamp = this->now();
		full_status.head.frame_id = "";

		full_status.node_name = this->name;
		full_status.parent = -1;
		full_status.parent_name = this->parent_name;

		full_status.child_name = this->child_names;
		full_status.params = this->logic->params;

		full_status.plugin_name = this->plugin_name;

		full_status.owned_resources = this->owned_resources;

		full_status.node_status = this->status;

		this->status_pub->publish(full_status);
	}

	void Node::set_resource_status_updated(bool to_set)
	{
		this->resource_status_updated = to_set;
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
		
		if ( this->status.state == dhtt_msgs::msg::NodeStatus::WAITING )
		{
			this->update_status(dhtt_msgs::msg::NodeStatus::ACTIVE);

			// spin up the auction activation thread
			this->work_thread = std::make_shared<std::thread>(&Node::activate, this, goal_handle);
			this->work_thread->detach();
		}
		else if ( this->status.state == dhtt_msgs::msg::NodeStatus::ACTIVE )
		{
			if ( goal_handle->get_goal()->success )
			{
				this->update_status(dhtt_msgs::msg::NodeStatus::WORKING);

				// spin up the thread for working and detach so that it destructs after it finishes
				this->work_thread = std::make_shared<std::thread>(&Node::activate, this, goal_handle);
				this->work_thread->detach();
			}
			else
			{
				RCLCPP_DEBUG(this->get_logger(), "Request not successful returning to state WAITING");

				this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

				this->resource_status_updated = false;
				
				dhtt_msgs::action::Activation::Result::SharedPtr blank_result = std::make_shared<dhtt_msgs::action::Activation::Result>();

				if ( (int) this->child_names.size() > 0 )
				{
					this->fail_thread.reset();

					this->fail_thread = std::make_shared<std::thread>(&Node::propogate_failure_down, this);
					this->fail_thread->detach();
				}

				goal_handle->succeed(blank_result);
			}
		}
		else if ( this->status.state == dhtt_msgs::msg::NodeStatus::DONE )
		{
			dhtt_msgs::action::Activation::Result::SharedPtr blank_result = std::make_shared<dhtt_msgs::action::Activation::Result>();
			blank_result->done = true;

			goal_handle->succeed(blank_result);
		}
	}

	void Node::store_result_callback( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Activation>::WrappedResult & result, std::string node_name )
	{
		this->responses[node_name] = result.result;
	
		this->stored_responses++;
	}

	// simpler than it could be especially with some work in interruption handling
	void Node::activate(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle)
	{
		// collect result from children or self
		dhtt_msgs::action::Activation::Result::SharedPtr to_ret;

		std::lock_guard<std::mutex> guard(this->logic_mut);

		this->owned_resources.clear();

		RCLCPP_INFO(this->get_logger(), "Activation received from parent...");

		if ( this->status.state == dhtt_msgs::msg::NodeStatus::ACTIVE )
		{
			// wait real quick for the status to be updated
			while (not this->resource_status_updated);

			RCLCPP_DEBUG(this->get_logger(), "Resource status updated activating callback");

			// deal with any passed resources
			for ( dhtt_msgs::msg::Resource passed_resource : goal_handle->get_goal()->passed_resources )
				this->owned_resources.push_back( passed_resource );

			// then start auction work
			to_ret = this->logic->auction_callback(this);

			this->active_child_name = to_ret->local_best_node;

			if (this->logic->is_done())
				this->update_status(dhtt_msgs::msg::NodeStatus::DONE);
		}
		else if ( this->status.state == dhtt_msgs::msg::NodeStatus::WORKING )
		{
			RCLCPP_WARN(this->get_logger(), "Starting work!");

			// add granted resources to the owned resources for the logic
			for ( dhtt_msgs::msg::Resource granted_resource : goal_handle->get_goal()->granted_resources )
			{
				// RCLCPP_WARN(this->get_logger(), "[%s]", granted_resource.name.c_str());
				this->owned_resources.push_back( granted_resource );
			}

			// start work 
			to_ret = this->logic->work_callback(this);
				
			this->resource_status_updated = false;

			if (this->logic->is_done())
				this->update_status(dhtt_msgs::msg::NodeStatus::DONE);
			else
				this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);
		}
		else if ( this->status.state == dhtt_msgs::msg::NodeStatus::DONE )
		{
			to_ret->done = true;
		}

		to_ret->activation_potential = this->logic->get_perceived_efficiency();
		// this->calculate_activation_potential();

		// then send result back to parent
		goal_handle->succeed(to_ret);
	}

	void Node::register_child_callback(std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Response> response)
	{
		// std::lock_guard<std::mutex> guard(this->logic_mut);

		if (not this->logic->can_add_child())
		{
			response->success = false;

			return;
		}

		// make a client and push back
		rclcpp_action::Client<dhtt_msgs::action::Activation>::SharedPtr n_client = rclcpp_action::create_client<dhtt_msgs::action::Activation>(this, std::string(TREE_PREFIX) + "/" + request->node_name + ACTIVATION_POSTFIX);

		this->child_names.push_back(request->node_name);
		this->activation_clients.push_back(n_client);
		response->success = true;
	}
	
	void Node::modify(std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response)
	{
		if ( request->type == dhtt_msgs::srv::ModifyRequest::Request::MUTATE )
		{
			std::lock_guard<std::mutex> guard(this->logic_mut);

			this->update_status(dhtt_msgs::msg::NodeStatus::MUTATING);

			std::shared_ptr<NodeType> temp_logic;

			try
			{
				temp_logic = node_type_loader.createSharedInstance(request->mutate_type);

				temp_logic->initialize(request->params);
			}
			catch (std::exception& ex)
			{
				response->error_msg = "Error when loading plugin " + request->mutate_type + ": " + ex.what();
				response->success = false;

				return;
			}

			if ( this->logic->can_add_child() != temp_logic->can_add_child() and ( (int) this->child_names.size() > 0 ) )
			{
				response->error_msg = "Error when processing mutation request: This node has children but the new plugin cannot have children. Returning in error.";
				response->success = false;

				return;
			}

			this->logic.reset();

			this->logic = temp_logic;

			this->plugin_name = request->mutate_type;

			response->error_msg = "";
			response->success = true;

			int8_t n_status = dhtt_msgs::msg::NodeStatus::WAITING;

			if ( this->status.state == dhtt_msgs::msg::NodeStatus::ACTIVE )
				n_status = dhtt_msgs::msg::NodeStatus::ACTIVE;

			this->update_status(n_status);

			return;
		}

		if ( request->type == dhtt_msgs::srv::ModifyRequest::Request::PARAM_UPDATE )
		{
			std::lock_guard<std::mutex> guard(this->logic_mut);

			this->logic->params.clear();

			try
			{
				this->logic->parse_params(request->params);
			}
			catch (std::exception& ex)
			{
				response->error_msg = std::string("Error parsing new params: ") + ex.what();
				response->success = false;

				return;
			}

			response->error_msg = "";
			response->success = true;

			this->update_status(this->status.state);

			return;
		}

		if (request->type == dhtt_msgs::srv::ModifyRequest::Request::REPARENT)
		{
			// TODO lock logic_mut?
			this->parent_name = request->new_parent;
			response->error_msg = "";
			response->success = true;
			// remove_child() on old parent and register_with_parent() on new parent is done by main server

			this->update_status(this->status.state);
		}
	}

	void Node::resource_availability_callback( const dhtt_msgs::msg::Resources::SharedPtr canonical_list )
	{
		this->available_resources = canonical_list->resource_state;

		this->resource_status_updated = true;
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