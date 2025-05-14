#include "dhtt/tree/node.hpp"

namespace dhtt
{
	Node::Node(std::string name, std::string type, std::vector<std::string> params, std::string p_name, std::string goitr_type) : rclcpp::Node( name ), conc_group(nullptr), 
			node_type_loader("dhtt", "dhtt::NodeType"), goitr_type_loader("dhtt", "dhtt::GoitrType"), name(name), parent_name(p_name), priority(1), resource_status_updated(false), first_activation(true), active(false)
	{
		// create a callback group for parallel ones
		this->conc_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		this->exclusive_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		// this->exclusive_maintenance_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->sub_opts.callback_group = this->conc_group;
		this->pub_opts.callback_group = this->conc_group;

		this->active_child_name = "";
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

		if ( strcmp(goitr_type.c_str(), "") )
		{
			try
			{
				this->replanner = goitr_type_loader.createSharedInstance(goitr_type);
			}
			catch (pluginlib::PluginlibException& ex)
			{
				this->error_msg = "Error when loading plugin " + goitr_type + ": " + ex.what();
				this->successful_load = false;

				return;
			}

			this->replanner->initialize(this->name, params);	
			this->has_goitr = true;
		}
		else
		{
			this->has_goitr = false;
		}

		this->plugin_name = type;
		this->goitr_name = goitr_type;

		this->logic->initialize(params);
	}

	Node::~Node()
	{
		// empty for now
		if ( this->has_goitr )	
			this->replanner->destruct();
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

	void Node::register_with_parent(int index)
	{
		// should only evaluate to false in the case of ROOT
		if ( strcmp(this->parent_name.c_str(), "NONE") )
		{
			std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> req = std::make_shared<dhtt_msgs::srv::InternalServiceRegistration::Request>();
			req->node_name = this->name;
			req->index = index;

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
		std::string my_condition_topic = std::string(TREE_PREFIX) + "/" + name + CONDITION_POSTFIX;
		std::string resources_topic = std::string(TREE_PREFIX) + RESOURCES_POSTFIX;

		this->status_pub = this->create_publisher<dhtt_msgs::msg::Node>("/status", 10, this->pub_opts);
		// this->knowledge_pub = this->create_publisher<std_msgs::msg::String>("/updated_knowledge", 10);

		this->resources_sub = this->create_subscription<dhtt_msgs::msg::Resources>(resources_topic, 10, std::bind(&Node::resource_availability_callback, this, std::placeholders::_1), this->sub_opts);

		this->register_server = this->create_service<dhtt_msgs::srv::InternalServiceRegistration>(my_register_topic, std::bind(&Node::register_child_callback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->conc_group);

		// set up activation server
		this->activation_server = rclcpp_action::create_server<dhtt_msgs::action::Activation>(this, my_activation_topic,
			std::bind(&Node::goal_activation_callback, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&Node::cancel_activation_callback, this, std::placeholders::_1),
			std::bind(&Node::activation_accepted_callback, this, std::placeholders::_1),
			rcl_action_server_get_default_options(), this->conc_group);

		// set up condition maintenance server
		this->condition_server = rclcpp_action::create_server<dhtt_msgs::action::Condition>(this, my_condition_topic,
			std::bind(&Node::condition_goal_activation_callback, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&Node::condition_cancel_activation_callback, this, std::placeholders::_1),
			std::bind(&Node::condition_activation_accepted_callback, this, std::placeholders::_1));
			// rcl_action_server_get_default_options(), this->exclusive_group);

		this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

		RCLCPP_INFO(this->get_logger(), "\tServers registered for node %s", this->name.c_str());
	}

	bool Node::remove_child(std::string child_name)
	{
		std::lock_guard<std::mutex> lock(this->maintenance_mut);
		auto found_name = [&]( std::string to_check ) { return not strcmp(to_check.c_str(), child_name.c_str()); };

		auto found_iter = std::find_if(this->child_names.begin(), this->child_names.end(), found_name);

		if ( found_iter == this->child_names.end() )
			return false;

		int found_index = std::distance(this->child_names.begin(), found_iter);

		this->activation_clients.erase(std::next(this->activation_clients.begin(), found_index));
		this->conditions_clients.erase(std::next(this->conditions_clients.begin(), found_index)); // hopefully these are always in the same order
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

	void Node::async_activate_child_failed(std::string child_name)
	{
		auto found_name = [&]( std::string to_check ) { return not strcmp(to_check.c_str(), child_name.c_str()); };

		auto found_iter = std::find_if(this->child_names.begin(), this->child_names.end(), found_name);
		int found_index = std::distance(this->child_names.begin(), found_iter);

		dhtt_msgs::action::Activation::Goal n_goal;

		n_goal.success = false;

		auto client_iter = std::next(this->activation_clients.begin(), found_index);

		if ( not (*client_iter)->wait_for_action_server() )
		{
			this->error_msg = "Cannot reach action server of requested child, are you sure it started up?";
			return;
		}

		this->expected_failed_responses++;

		auto send_goal_options = rclcpp_action::Client<dhtt_msgs::action::Activation>::SendGoalOptions();

		send_goal_options.result_callback = std::bind(&Node::store_failed_callback, this, std::placeholders::_1, child_name);

		(*client_iter)->async_send_goal(n_goal, send_goal_options);
	}

	void Node::async_request_conditions(std::string child_name, dhtt_msgs::action::Condition::Goal condition_goal)
	{
		auto found_name = [&]( std::string to_check ) { return not strcmp(to_check.c_str(), child_name.c_str()); };

		auto found_iter = std::find_if(this->child_names.begin(), this->child_names.end(), found_name);
		int found_index = std::distance(this->child_names.begin(), found_iter);

		auto client_iter = std::next(this->conditions_clients.begin(), found_index);

		if ( not (*client_iter)->wait_for_action_server() )
		{
			this->error_msg = "Cannot reach action server of requested child, are you sure it started up?";
			return;
		}

		this->expected_conditions++;

		auto send_goal_options = rclcpp_action::Client<dhtt_msgs::action::Condition>::SendGoalOptions();

		send_goal_options.result_callback = std::bind(&Node::store_condition_callback, this, std::placeholders::_1, child_name);

		(*client_iter)->async_send_goal(condition_goal, send_goal_options);
	}

	void Node::activate_all_children(dhtt_msgs::action::Activation::Goal activation_goal)
	{
		for (std::string iter : this->child_names)
			this->async_activate_child(iter, activation_goal);
	}
	
	void Node::request_conditions_from_children()
	{
		dhtt_msgs::action::Condition::Goal n_goal;
		n_goal.type = dhtt_msgs::action::Condition::Goal::MAINTAIN;
		this->stored_conditions = 0;
		this->expected_conditions = 0;

		for (std::string iter : this->child_names)
		{
			while ( not this->server_ready[iter] );
			this->async_request_conditions(iter, n_goal);
		}
	}

	bool Node::block_for_responses_from_children()
	{
		// RCLCPP_INFO(this->get_logger(), "Waiting for %d responses from children, currently received %d...", this->expected_responses, this->stored_responses );

		while (this->stored_responses < this->expected_responses);

		// RCLCPP_INFO(this->get_logger(), "Responses received!");

		return true;
	}

	bool Node::block_for_responses_from_failed_children()
	{
		RCLCPP_DEBUG(this->get_logger(), "Waiting for %d responses from children, currently received %d...", this->expected_responses, this->stored_responses );

		while (this->stored_failed_responses < this->expected_failed_responses);

		// RCLCPP_INFO(this->get_logger(), "Responses received!");

		return true;
	}
	
	bool Node::block_for_conditions_from_children()
	{
		while (this->stored_conditions < this->expected_conditions);

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

	std::map<std::string, dhtt_msgs::action::Condition::Result::SharedPtr> Node::get_condition_results()
	{
		// reset values
		this->stored_conditions = 0;
		this->expected_conditions = 0;

		std::map<std::string, dhtt_msgs::action::Condition::Result::SharedPtr> to_ret;

		for ( auto const& x : this->child_conditions )
			to_ret[x.first] = x.second;

		this->child_conditions.clear();

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
		full_status.goitr_name = this->goitr_name;

		full_status.owned_resources = this->owned_resources;

		full_status.node_status = this->status;

		std::shared_ptr<dhtt_utils::PredicateConjunction> tmp1 = std::make_shared<dhtt_utils::PredicateConjunction>(); 
		std::shared_ptr<dhtt_utils::PredicateConjunction> tmp2 = std::make_shared<dhtt_utils::PredicateConjunction>();

		*tmp1 = this->logic->get_preconditions();
		*tmp2 = this->logic->get_postconditions();

		full_status.preconditions = dhtt_utils::to_string(tmp1);
		full_status.postconditions = dhtt_utils::to_string(tmp2);

		this->status_pub->publish(full_status);
	}

	void Node::fire_knowledge_updated()
	{
		std_msgs::msg::String n_msg;

		this->knowledge_pub->publish(n_msg);
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
				RCLCPP_INFO(this->get_logger(), "Request not successful returning to state WAITING.");

				this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

				this->resource_status_updated = false;
				
				dhtt_msgs::action::Activation::Result::SharedPtr blank_result = std::make_shared<dhtt_msgs::action::Activation::Result>();

				if ( (int) this->child_names.size() > 0 and this->active )
				{
					this->fail_thread.reset();

					this->fail_thread = std::make_shared<std::thread>(&Node::propogate_failure_down, this);
					this->fail_thread->detach();
				}

				goal_handle->succeed(blank_result);
				this->active = false;
			}
		}
		else if ( this->status.state == dhtt_msgs::msg::NodeStatus::DONE )
		{
			dhtt_msgs::action::Activation::Result::SharedPtr blank_result = std::make_shared<dhtt_msgs::action::Activation::Result>();
			blank_result->done = true;

			goal_handle->succeed(blank_result);
		}
	}

	rclcpp_action::GoalResponse Node::condition_goal_activation_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const dhtt_msgs::action::Condition::Goal> goal)
	{
		RCLCPP_DEBUG(this->get_logger(), "Received condition request.");
		(void) uuid;
		(void) goal;

		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse Node::condition_cancel_activation_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Condition>> goal_handle)
	{

		RCLCPP_DEBUG(this->get_logger(), "Received request to cancel condition request.");
		(void) goal_handle;

		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void Node::condition_activation_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Condition>> goal_handle)
	{
		auto type = goal_handle->get_goal()->type;

		if ( type == dhtt_msgs::action::Condition::Goal::MAINTAIN )
		{
			this->condition_thread = std::make_shared<std::thread>(&Node::combine_child_conditions, this, goal_handle);
			this->condition_thread->detach();
		}
		else if ( type == dhtt_msgs::action::Condition::Goal::GET )
		{
			dhtt_msgs::action::Condition::Result::SharedPtr to_ret = std::make_shared<dhtt_msgs::action::Condition::Result>();

			to_ret->success = true;

			std::shared_ptr<dhtt_utils::PredicateConjunction> tmp1 = std::make_shared<dhtt_utils::PredicateConjunction>();
			std::shared_ptr<dhtt_utils::PredicateConjunction> tmp2 = std::make_shared<dhtt_utils::PredicateConjunction>();

			*tmp1 = this->logic->get_preconditions();
			*tmp2 = this->logic->get_postconditions(); 

			to_ret->preconditions = dhtt_utils::to_string(tmp1);
			to_ret->postconditions = dhtt_utils::to_string(tmp2);

			goal_handle->succeed(to_ret);
		}
		else
		{
			dhtt_msgs::action::Condition::Result::SharedPtr to_ret = std::make_shared<dhtt_msgs::action::Condition::Result>();

			to_ret->success = false;
			to_ret->error_msg = "Unknown action " + std::to_string(type) + " requested... returning in error.";

			goal_handle->succeed(to_ret);
		}
	}

	void Node::combine_child_conditions(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Condition>> goal_handle)
	{
		// RCLCPP_WARN(this->get_logger(), "Requesting conditions from children and combining!");
		std::lock_guard<std::mutex> guard(this->maintenance_mut);

		(void) goal_handle->get_goal();

		if ( (int) this->child_names.size() > 0 )
		{
			this->request_conditions_from_children();

			this->block_for_conditions_from_children();
		}

		this->logic->maintain_conditions(this);

		dhtt_msgs::action::Condition::Result::SharedPtr to_ret = std::make_shared<dhtt_msgs::action::Condition::Result>();

		to_ret->success = true;
		
		std::shared_ptr<dhtt_utils::PredicateConjunction> tmp1 = std::make_shared<dhtt_utils::PredicateConjunction>();
		std::shared_ptr<dhtt_utils::PredicateConjunction> tmp2 = std::make_shared<dhtt_utils::PredicateConjunction>();

		*tmp1 = this->logic->get_preconditions();
		*tmp2 = this->logic->get_postconditions(); 

		to_ret->preconditions = dhtt_utils::to_string(tmp1);
		to_ret->postconditions = dhtt_utils::to_string(tmp2);

		// clear the responses just to be sure
		(void) this->get_condition_results();

		this->update_status(this->status.state);

		goal_handle->succeed(to_ret);

		// RCLCPP_INFO(this->get_logger(), "%s", this->name.c_str());
	}

	void Node::store_result_callback( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Activation>::WrappedResult & result, std::string node_name )
	{
		// TODO if (not result.result->possible)
		this->responses[node_name] = result.result;
	
		this->stored_responses++;
	}

	void Node::store_failed_callback( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Activation>::WrappedResult & result, std::string node_name )
	{
		// TODO if (not result.result->possible)
		// this->responses[node_name] = result.result;
		(void) result;
		(void) node_name;

		this->stored_failed_responses++;
	}

	void Node::store_condition_callback ( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Condition>::WrappedResult & result, std::string node_name )
	{
		// RCLCPP_ERROR(this->get_logger(), "Got condition result from child %s [%d]", node_name.c_str(), this->stored_conditions);
		this->child_conditions[node_name] = result.result;

		this->stored_conditions++;
	}

	// simpler than it could be especially with some work in interruption handling
	void Node::activate(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle)
	{
		// collect result from children or self
		dhtt_msgs::action::Activation::Result::SharedPtr to_ret = std::make_shared<dhtt_msgs::action::Activation::Result>();

		this->owned_resources.clear();

		RCLCPP_INFO(this->get_logger(), "Activation received from parent...");

		// this->block_for_responses_from_failed_children();

		// if ( this->has_goitr )
		// 	this->replanner->block_for_thread();

		// if ( this->has_goitr and this->first_activation )
		// {
		// 	RCLCPP_DEBUG(this->get_logger(), "This is first activation so building subtree...");
		// 	this->replanner->first_activation_callback();

		// 	to_ret->done = false;
		// 	to_ret->possible = false;
			
		// 	// this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);
		// 	this->first_activation = false;
		// }
		// else

		{
			std::lock_guard<std::mutex> guard(this->logic_mut);

			if ( this->status.state == dhtt_msgs::msg::NodeStatus::ACTIVE )
			{
				// wait real quick for the status to be updated
				// while (not this->resource_status_updated);

				// RCLCPP_FATAL(this->get_logger(), "Resource status updated activating callback");

				// deal with any passed resources
				for ( dhtt_msgs::msg::Resource passed_resource : goal_handle->get_goal()->passed_resources )
					this->owned_resources.push_back( passed_resource );

				// then start auction work
				to_ret = this->logic->auction_callback(this);

				this->active_child_name = to_ret->local_best_node;

				// check preconditions before moving request up
				to_ret->possible = to_ret->possible and this->check_preconditions();

				if ( to_ret->possible )
					this->active = true;
				else
					this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

				if (this->logic->is_done())
					this->update_status(dhtt_msgs::msg::NodeStatus::DONE);
			}
			else if ( this->status.state == dhtt_msgs::msg::NodeStatus::WORKING )
			{
				RCLCPP_WARN(this->get_logger(), "Starting work!");

				// add granted resources to the owned resources for the logic
				for ( dhtt_msgs::msg::Resource granted_resource : goal_handle->get_goal()->granted_resources )
					this->owned_resources.push_back( granted_resource );

				// start work as long as preconditions are met
				if ( this->check_preconditions() )
				{
					to_ret = this->logic->work_callback(this);

					this->apply_postconditions();
				}
				else
					RCLCPP_ERROR(this->get_logger(), "Preconditions not met, restarting auction!");

				this->resource_status_updated = false;

				if (this->logic->is_done())
					this->update_status(dhtt_msgs::msg::NodeStatus::DONE);
				else
					this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

				if ( strcmp("", to_ret->last_behavior.c_str() ) and this->has_goitr )
				{
					RCLCPP_DEBUG(this->get_logger(), "Child finished, running fault recovery callback"); 

					this->replanner->child_finished_callback(to_ret->last_behavior, to_ret->done);

					to_ret->last_behavior = "";
				}
			}
			else if ( this->status.state == dhtt_msgs::msg::NodeStatus::DONE )
			{
				to_ret->done = true;
			}
		}

		to_ret->activation_potential = this->logic->get_perceived_efficiency();
		// this->calculate_activation_potential();

		// then send result back to parent
		RCLCPP_INFO(this->get_logger(), "Sending request back up the tree...");
		goal_handle->succeed(to_ret);
	}

	void Node::register_child_callback(std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Response> response)
	{
		std::lock_guard<std::mutex> guard(this->logic_mut);
		std::lock_guard<std::mutex> maintenance_guard(this->maintenance_mut);

		if (not this->logic->can_add_child())
		{
			response->success = false;

			return;
		}

		this->server_ready[request->node_name] = false;

		// make a client and push back
		rclcpp_action::Client<dhtt_msgs::action::Activation>::SharedPtr a_client = rclcpp_action::create_client<dhtt_msgs::action::Activation>(this, std::string(TREE_PREFIX) + "/" + request->node_name + ACTIVATION_POSTFIX);
		rclcpp_action::Client<dhtt_msgs::action::Condition>::SharedPtr c_client = rclcpp_action::create_client<dhtt_msgs::action::Condition>(this, std::string(TREE_PREFIX) + "/" + request->node_name + CONDITION_POSTFIX);

		auto name_index_iter = (request->index == -1)? this->child_names.end() : this->child_names.begin() + request->index;
		auto activation_index_iter = (request->index == -1)? this->activation_clients.end() : this->activation_clients.begin() + request->index;
		auto condition_index_iter = (request->index == -1)? this->conditions_clients.end() : this->conditions_clients.begin() + request->index;

		this->child_names.insert(name_index_iter, request->node_name);
		this->activation_clients.insert(activation_index_iter, a_client);
		this->conditions_clients.insert(condition_index_iter, c_client);
		this->server_ready[request->node_name] = true;
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

				RCLCPP_ERROR(this->get_logger(), "Error parsing new params: %s", ex.what());

				throw ex; 

				return;
			}

			response->error_msg = "";
			response->success = true;

			int8_t state = dhtt_msgs::msg::NodeStatus::WAITING;

			// if ( state == dhtt_msgs::msg::NodeStatus::DONE )
				

			this->update_status(state);

			return;
		}

	}

	void Node::resource_availability_callback( const dhtt_msgs::msg::Resources::SharedPtr canonical_list )
	{
		RCLCPP_DEBUG(this->get_logger(), "Resources updated");
		this->available_resources = canonical_list->resource_state;

		this->resource_status_updated = true;
	}

	bool Node::check_preconditions()
	{
		// std::vector<std::vector<std::string>> preconditions = this->logic->get_preconditions();

		// for ( auto iter : preconditions )
		// {
		// 	// pull corresponding value from the param server

		// 	// check against value from preconditions
		// 	if ( false )
		// 		return false;
		// }

		return true;
	}

	void Node::apply_postconditions()
	{
		// set parameters on the param server
	}

	double Node::calculate_activation_potential()
	{
		return this->logic->get_perceived_efficiency() * (this->priority + (int) this->owned_resources.size());
	}

	void Node::propogate_failure_down()
	{
		// std::lock_guard<std::mutex> logic_lock(this->logic_mut); 

		// TODO occasionally causes a segfault, not sure if this is the right solution
		if (not this->active_child_name.empty())
		{
			this->async_activate_child_failed(this->active_child_name);
			this->block_for_responses_from_failed_children();
		}

		this->active_child_name = "";
	}
}