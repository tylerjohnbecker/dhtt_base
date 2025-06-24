#include "dhtt/tree/node.hpp"

namespace dhtt
{
	Node::Node(std::shared_ptr<CommunicationAggregator> com_agg, std::string name, std::string type, std::vector<std::string> params, std::string p_name, std::string branch_socket_type, std::string goitr_type) : rclcpp::Node( name ), conc_group(nullptr), 
			node_type_loader("dhtt", "dhtt::NodeType"), goitr_type_loader("dhtt", "dhtt::GoitrType"), 
			branch_socket_type_loader("dhtt", "dhtt::BranchSocketType"), branch_plug_type_loader("dhtt", "dhtt::BranchPlugType"),
			name(name), parent_name(p_name), priority(1), resource_status_updated(false), first_activation(true), active(false)
	{
		// create a callback group for parallel ones
		this->conc_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		this->sub_opts.callback_group = this->conc_group;
		this->pub_opts.callback_group = this->conc_group;

		this->active_child_name = "";
		this->error_msg = "";
		this->successful_load = true;

		this->global_com = com_agg;

		try
		{
			this->logic = this->node_type_loader.createSharedInstance(type);
		}
		catch (pluginlib::PluginlibException& ex)
		{
			this->error_msg = "Error when loading plugin " + type + ": " + ex.what();
			this->successful_load = false;

			RCLCPP_ERROR(this->get_logger(), "%s", this->error_msg.c_str());

			return;
		}

		try
		{
			this->parent_communication_socket = this->branch_socket_type_loader.createSharedInstance(branch_socket_type);

			this->parent_communication_socket->initialize(this);
		}
		catch (pluginlib::PluginlibException& ex)
		{
			this->error_msg = "Error when loading plugin " + branch_socket_type + ": " + ex.what();
			this->successful_load = false;

			RCLCPP_ERROR(this->get_logger(), "%s", this->error_msg.c_str());

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
		this->socket_name = branch_socket_type;
		this->goitr_name = goitr_type;

		this->logic->com_agg = this->global_com;
		this->logic->initialize(params);
	}

	Node::~Node()
	{
		// empty for now
		if ( this->has_goitr )	
			this->replanner->destruct();

		this->parent_communication_socket->destruct();

		for ( const auto& pair : this->child_communication_plugs ) // std::pair< std::string, std::pair< bool , std::shared_ptr<BranchPlugType>>>
			pair.second.second->destruct();

		std::string resources_topic = std::string(TREE_PREFIX) + RESOURCES_POSTFIX;

		this->global_com->unregister_subscription<dhtt_msgs::msg::Resources>(resources_topic, this->name);
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

	std::shared_ptr<BranchSocketType> Node::get_socket_ptr()
	{
		return this->parent_communication_socket;
	}

	std::shared_ptr<CommunicationAggregator> Node::get_com_agg()
	{
		return this->global_com;
	}

	void Node::set_passed_resources(std::vector<dhtt_msgs::msg::Resource> set_to)
	{
		// this is an unsafe method for now. USE AT YOUR OWN DISCRETION
		this->passed_resources.clear();

		this->passed_resources = set_to;
	}

	void Node::register_servers()
	{
		// set up services and action servers
		std::string resources_topic = std::string(TREE_PREFIX) + RESOURCES_POSTFIX;
		
		this->status_pub = this->global_com->register_publisher<dhtt_msgs::msg::Node>("/status");

		// this->status_pub = this->create_publisher<dhtt_msgs::msg::Node>("/status", 10, this->pub_opts);
		// this->knowledge_pub = this->create_publisher<std_msgs::msg::String>("/updated_knowledge", 10);

		this->global_com->register_subscription<dhtt_msgs::msg::Resources>(resources_topic, this->name, std::bind(&Node::resource_availability_callback, this, std::placeholders::_1) );
		// this->resources_sub = this->create_subscription<dhtt_msgs::msg::Resources>(resources_topic, 10, std::bind(&Node::resource_availability_callback, this, std::placeholders::_1), this->sub_opts);

		this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

		RCLCPP_INFO(this->get_logger(), "\tServers registered for node %s", this->name.c_str());
	}

	bool Node::add_child(std::shared_ptr<BranchSocketType> socket_ptr, std::string child_name, std::string plug_type, int index)
	{

		std::lock_guard<std::mutex> guard(this->logic_mut);
		std::lock_guard<std::mutex> maintenance_guard(this->maintenance_mut);

		if (not this->logic->can_add_child())
		{
			this->error_msg = "This node cannot add children.";

			return false;
		}

		std::shared_ptr<BranchPlugType> n_plug_ptr;

		try
		{
			n_plug_ptr = this->branch_plug_type_loader.createSharedInstance(plug_type);

			n_plug_ptr->initialize(this, socket_ptr, child_name);
		}
		catch (pluginlib::PluginlibException& ex)
		{
			this->error_msg = "Error when loading plugin " + plug_type + ": " + ex.what();

			return false;
		}

		std::pair<bool, std::shared_ptr<BranchPlugType>> n_pair{false, n_plug_ptr};

		this->child_communication_plugs[child_name] = n_pair;

		auto name_index_iter = (index == -1)? this->child_names.end() : this->child_names.begin() + index;
		this->child_names.insert(name_index_iter, child_name);

		return true;
	}

	bool Node::remove_child(std::string child_name)
	{
		std::lock_guard<std::mutex> lock(this->maintenance_mut);

		auto found_name = [&]( std::string to_check ) { return not strcmp(to_check.c_str(), child_name.c_str()); };

		auto found_iter = std::find_if(this->child_names.begin(), this->child_names.end(), found_name);

		if ( found_iter == this->child_names.end() )
			return false;

		this->child_communication_plugs[child_name].second->destruct();
		this->child_communication_plugs.erase(child_name);

		this->child_names.erase(found_iter);

		return true;
	}
	
	bool Node::async_activate_child(std::string child_name, dhtt_msgs::action::Activation::Goal activation_goal)
	{
		if ( this->child_communication_plugs[child_name].first or not this->child_communication_plugs[child_name].second->async_send_activate(activation_goal) )
			return false;

		this->child_communication_plugs[child_name].first = true;
		return true;
	}

	bool Node::async_request_conditions(std::string child_name, dhtt_msgs::action::Condition::Goal condition_goal)
	{
		RCLCPP_INFO(this->get_logger(), "Requesting conditions from [%s]", child_name.c_str());

		return this->child_communication_plugs[child_name].second->async_send_maintain(condition_goal);
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

		for (std::string iter : this->child_names)
			this->async_request_conditions(iter, n_goal);
	}

	bool Node::block_for_activation_from_children()
	{
		for ( auto& pair : this->child_communication_plugs )
			if ( pair.second.first )
				pair.second.second->block_for_activation_response();

		return true;
	}
	
	bool Node::block_for_conditions_from_children()
	{
		for ( const auto& pair : this->child_communication_plugs )
			pair.second.second->block_for_maintain_response();

		return true;
	}

	std::map<std::string, dhtt_msgs::action::Activation::Result> Node::get_activation_results()
	{	
		std::map<std::string, dhtt_msgs::action::Activation::Result> to_ret;

		for ( auto iter : this->child_names )
		{
			if ( this->child_communication_plugs[iter].first )
			{
				dhtt_msgs::action::Activation::Result res = this->child_communication_plugs[iter].second->get_activation_result();
				this->child_communication_plugs[iter].first = false;

				// if ( res == dhtt_msgs::action::Activation::Result() )
				// 	RCLCPP_ERROR(this->get_logger(), "Received empty activation message from [%s]", iter.c_str());

				to_ret[iter] = res;
			}
		}

		return to_ret;
	}

	std::map<std::string, dhtt_msgs::action::Condition::Result> Node::get_condition_results()
	{
		std::map<std::string, dhtt_msgs::action::Condition::Result> to_ret;

		for ( auto iter : this->child_names )
		{
			dhtt_msgs::action::Condition::Result res = this->child_communication_plugs[iter].second->get_maintain_result();

			if ( res != dhtt_msgs::action::Condition::Result() )
				to_ret[iter] = res;
		}

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
		// std_msgs::msg::String n_msg;

		// this->knowledge_pub->publish(n_msg);
	}

	void Node::set_resource_status_updated(bool to_set)
	{
		this->resource_status_updated = to_set;
	}


	// simpler than it could be especially with some work in interruption handling
	dhtt_msgs::action::Activation::Result Node::activate(dhtt_msgs::action::Activation::Goal goal)
	{
		// collect result from children or self
		dhtt_msgs::action::Activation::Result to_ret;

		this->owned_resources.clear();

		RCLCPP_INFO(this->get_logger(), "Activation received from parent...");

		// I think this is messing with the fsm related to the branch_types
		// this->block_for_activation_from_children();

		// we should get one resource update per message up the tree at least
		while (not this->resource_status_updated);

		std::lock_guard<std::mutex> guard(this->logic_mut);

		if ( this->status.state == dhtt_msgs::msg::NodeStatus::WAITING )
		{
			this->update_status(dhtt_msgs::msg::NodeStatus::ACTIVE);

			// deal with any passed resources
			for ( dhtt_msgs::msg::Resource passed_resource : goal.passed_resources )
				this->owned_resources.push_back( passed_resource );

			// then start auction work
			to_ret = *this->logic->auction_callback(this);

			this->active_child_name = to_ret.local_best_node;

			// check preconditions before moving request up
			to_ret.possible = to_ret.possible and this->check_preconditions();
			to_ret.activation_potential = this->logic->get_perceived_efficiency();

			if ( to_ret.possible )
				this->active = true;
			else
				this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

			if (this->logic->is_done())
				this->update_status(dhtt_msgs::msg::NodeStatus::DONE);
		}
		else if ( this->status.state == dhtt_msgs::msg::NodeStatus::ACTIVE )
		{
			if ( goal.success )
			{
				RCLCPP_WARN(this->get_logger(), "Starting work!");
				this->update_status(dhtt_msgs::msg::NodeStatus::WORKING);

				// add granted resources to the owned resources for the logic
				for ( dhtt_msgs::msg::Resource granted_resource : goal.granted_resources )
					this->owned_resources.push_back( granted_resource );

				// start work as long as preconditions are met
				if ( this->check_preconditions() )
				{
					to_ret = *this->logic->work_callback(this);

					this->apply_postconditions();
				}
				else
					RCLCPP_ERROR(this->get_logger(), "Preconditions not met, restarting auction!");

				this->resource_status_updated = false;

				if (this->logic->is_done())
					this->update_status(dhtt_msgs::msg::NodeStatus::DONE);
				else
					this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

				if ( strcmp("", to_ret.last_behavior.c_str() ) and this->has_goitr )
				{
					RCLCPP_DEBUG(this->get_logger(), "Child finished, running fault recovery callback"); 

					this->replanner->child_finished_callback(to_ret.last_behavior, to_ret.done);

					to_ret.last_behavior = "";
				}
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "Request not successful returning to state WAITING.");

				this->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

				this->resource_status_updated = false;

				// propogate failure down the tree
				if ( (int) this->child_names.size() > 0 )
				{
					this->child_communication_plugs[this->active_child_name].second->async_send_activate(goal);
					this->child_communication_plugs[this->active_child_name].second->block_for_activation_response();
					(void) this->child_communication_plugs[this->active_child_name].second->get_activation_result();
				}
			}

			return to_ret; 
		}
		else if ( this->status.state == dhtt_msgs::msg::NodeStatus::DONE )
		{
			to_ret.done = true;
		}

		// this->resource_status_updated = false;

		// then send result back to parent
		RCLCPP_INFO(this->get_logger(), "Sending request back up the tree...");
		return to_ret;
	}

	dhtt_msgs::action::Condition::Result Node::combine_child_conditions(dhtt_msgs::action::Condition::Goal goal)
	{
		dhtt_msgs::action::Condition::Result to_ret;
	
		if ( goal.type == dhtt_msgs::action::Condition::Goal::GET )
		{
			to_ret.success = true;

			std::shared_ptr<dhtt_utils::PredicateConjunction> tmp1 = std::make_shared<dhtt_utils::PredicateConjunction>();
			std::shared_ptr<dhtt_utils::PredicateConjunction> tmp2 = std::make_shared<dhtt_utils::PredicateConjunction>();

			*tmp1 = this->logic->get_preconditions();
			*tmp2 = this->logic->get_postconditions(); 

			to_ret.preconditions = dhtt_utils::to_string(tmp1);
			to_ret.postconditions = dhtt_utils::to_string(tmp2);

			return to_ret;
		}

		else if ( goal.type == dhtt_msgs::action::Condition::Goal::MAINTAIN)
		{

			std::lock_guard<std::mutex> guard(this->maintenance_mut);

			if ( (int) this->child_names.size() > 0 )
			{
				this->request_conditions_from_children();

				this->block_for_conditions_from_children();
			}

			this->logic->maintain_conditions(this);

			to_ret.success = true;
			
			std::shared_ptr<dhtt_utils::PredicateConjunction> tmp1 = std::make_shared<dhtt_utils::PredicateConjunction>();
			std::shared_ptr<dhtt_utils::PredicateConjunction> tmp2 = std::make_shared<dhtt_utils::PredicateConjunction>();

			*tmp1 = this->logic->get_preconditions();
			*tmp2 = this->logic->get_postconditions(); 

			to_ret.preconditions = dhtt_utils::to_string(tmp1);
			to_ret.postconditions = dhtt_utils::to_string(tmp2);

			// outputing updated preconditions on status pub
			this->update_status(this->status.state);

			return to_ret;
		}

		to_ret.success = false;
		to_ret.error_msg = "Unknown action " + std::to_string(goal.type) + " requested... returning in error.";

		return to_ret; 
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

				temp_logic->com_agg = this->global_com;
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

			this->update_status(state);

			return;
		}

	}

	void Node::resource_availability_callback( const dhtt_msgs::msg::Resources::SharedPtr canonical_list )
	{
		RCLCPP_FATAL(this->get_logger(), "Resources updated");
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
}