#include "dhtt/server/main_server.hpp"

namespace dhtt
{
	MainServer::MainServer(std::string node_name, std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner, bool slow) : 
									rclcpp::Node(node_name),//, rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)), 
									conc_group(nullptr), spinner_cp(spinner), total_nodes_added(1), verbose(true), running(false)
	{
		// create a callback group for parallel ones
		this->conc_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		this->sub_opts.callback_group = this->conc_group;
		this->pub_opts.callback_group = this->conc_group;

		/// initialize subscribers
		this->status_sub = this->create_subscription<dhtt_msgs::msg::Node>("/status", MAX_NODE_NUM, std::bind(&MainServer::status_callback, this, std::placeholders::_1), this->sub_opts);

		/// initialize publishers
		this->root_status_pub = this->create_publisher<dhtt_msgs::msg::NodeStatus>("/root_status", 10, this->pub_opts);

		/// initialize Global CommunicationAggregator
		this->global_com = std::make_shared<CommunicationAggregator>(spinner);
		this->spinner_cp->add_node(this->global_com);

		/// make root node (before external services because they will not work without an active root)
		// make local representation
		dhtt_msgs::msg::Node::SharedPtr root_node = std::make_shared<dhtt_msgs::msg::Node>();

		root_node->node_name = "ROOT_0";
		root_node->parent = ROOT_PARENT;
		root_node->parent_name = std::string("NONE");

		root_node->children = std::vector<int>();
		root_node->child_name = std::vector<std::string>();

		// pretty sure I'll switch this to a passed in parameter, but for now this should work
		dhtt_folder_path = __FILE__;

		dhtt_folder_path = dhtt_folder_path.parent_path().parent_path().parent_path();

		root_node->params.push_back( std::string("path: ") + dhtt_folder_path.native() + "/robots/pr2.yaml" );

		if ( slow )
		{
			root_node->params.push_back("GoSlow");
			RCLCPP_WARN(this->get_logger(), "Starting in testing mode...");
		}

		root_node->type = dhtt_msgs::msg::Node::ROOT;

		root_node->node_status.state = dhtt_msgs::msg::NodeStatus::WAITING;

		this->node_list.tree_nodes.push_back(*root_node);
		this->node_list.tree_status = dhtt_msgs::msg::NodeStatus::WAITING;
		this->node_list.max_tree_depth = 0;
		this->node_list.max_tree_width = 1;

		this->node_list.task_completion_percent = 0.0f;

		// initialize physical Root Node. loaded a yaml file as a part of the constructor so we don't catch it and make the program fail to load if that happens
		this->node_map["ROOT_0"] = std::make_shared<dhtt::Node>(this->global_com, "ROOT_0", "dhtt_plugins::RootBehavior", root_node->params, "NONE", "dhtt_plugins::PtrBranchSocket");
		// this->spinner_cp->add_node(this->node_map["ROOT_0"]);
		this->node_map["ROOT_0"]->register_servers();
		this->node_map["ROOT_0"]->set_resource_status_updated(true);
		
		// this->node_map["ROOT_0"]->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

		/// initialize external services
		this->modify_server = this->create_service<dhtt_msgs::srv::ModifyRequest>("/modify_service", std::bind(&MainServer::modify_callback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->conc_group);
		this->control_server = this->create_service<dhtt_msgs::srv::ControlRequest>("/control_service", std::bind(&MainServer::control_callback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->conc_group);
		this->fetch_server = this->create_service<dhtt_msgs::srv::FetchRequest>("/fetch_service", std::bind(&MainServer::fetch_callback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->conc_group);
		this->history_server = this->create_service<dhtt_msgs::srv::HistoryRequest>("/history_service", std::bind(&MainServer::history_callback, this,std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->conc_group);
		this->resource_server = this->create_service<dhtt_msgs::srv::ResourceRequest>("/resource_service", std::bind(&MainServer::resource_callback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->conc_group);

		// this->client_ptr = rclcpp_action::create_client<dhtt_msgs::action::Activation>( this, "/dhtt/ROOT_0/activate", this->conc_group);
		// this->maintenance_client_ptrs["ROOT_0"] = rclcpp_action::create_client<dhtt_msgs::action::Condition>(this, "/dhtt/ROOT_0/condition", this->conc_group);
		this->internal_control_client = this->create_client<dhtt_msgs::srv::InternalControlRequest>("/dhtt/control");

		// initialize and start maintenance thread
		this->maintenance_thread = std::make_shared<std::thread>( std::bind( &MainServer::maintainer_thread_cb, this ) );
		this->maintenance_thread->detach();

		this->resource_update_mut_ptr = this->global_com->request_mutex("resource_update");
	}

	MainServer::~MainServer()
	{
		// just manually cleaning up the data structure here
		this->verbose = false;

		// remove everyone's knowledge of their children first to avoid the severe warning from pluginlib
		for ( auto& pair : this->node_map )
		{
			auto children = pair.second->get_child_names();

			for ( auto name : children )
				pair.second->remove_child(name);
		}

		this->node_list.tree_nodes.clear();
		this->node_map.clear();

		this->end = true;

		this->maintenance_queue_condition.notify_one();
	}

	void MainServer::modify_callback( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response )
	{
		bool force = request->force;

		RCLCPP_INFO(this->get_logger(), "--- Modify request received!");
		this->publish_root_status();

		std::string to_remove;

		auto same_name = [&](std::string to_check) { return not strcmp(to_remove.c_str(), to_check.c_str()); };

		auto add_modify = [&](std::vector<std::string> to_add) {

			for ( auto iter : to_add )
				this->being_modified.push_back(iter);
		};

		auto remove_modify = [&](std::vector<std::string> rem) {

			for ( auto iter : rem )
			{
				to_remove = iter;

				this->being_modified.erase(std::remove_if(this->being_modified.begin(), this->being_modified.end(), same_name), this->being_modified.end());
			}
		};

		if ( (int) request->to_modify.size() == 0 )
		{
			response->success = false;
			response->error_msg = "Failed to modify node because no node was given."; 

			RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

			return;
		}

		for ( auto iter : request->to_modify )
		{
			if ( not this->can_modify(iter) )
			{
				response->success = false;
				response->error_msg = "Node " + iter + " is currently being modified... returning in error!";

				RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

				return;
			}
		}


		if ( request->type == dhtt_msgs::srv::ModifyRequest::Request::ADD)
		{
			{
				std::lock_guard<std::mutex> guard(this->modify_mut);

				add_modify(request->to_modify);

				for (auto iter = request->to_modify.begin(); iter != request->to_modify.end(); iter++)
				{
					if ( request->index < 0 )
						request->index = -1;

					response->error_msg = this->add_node(response, *iter, request->add_node, force, request->index);
					response->success = not strcmp(response->error_msg.c_str(), "");

					// exit early if modification fails
					if ( not response->success )
					{
						RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());
						remove_modify(request->to_modify);

						return;
					}
				}

				remove_modify(request->to_modify);
			}

			int id = this->start_maintain("ROOT_0");
			this->wait_for_maintain(id);

			return;
		}

		if ( request->type == dhtt_msgs::srv::ModifyRequest::Request::ADD_FROM_FILE)
		{
			add_modify(request->to_modify);

			for (auto iter = request->to_modify.begin(); iter != request->to_modify.end(); iter++)
			{
				// currently this opens the yaml file n times where n is the number of places that we add the nodes
				// in practice we should have negligible time loss from this but it should technically be done first before the loop
				// and then the dictionary should be passed in
				response->error_msg = this->add_nodes_from_file(response, *iter, request->to_add, request->file_args, force);
				response->success = not strcmp(response->error_msg.c_str(), "");

				// exit early if modification fails
				if ( not response->success )
				{
					RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());
					remove_modify(request->to_modify);

					return;
				}
			}

			remove_modify(request->to_modify);

			return;
		}

		if ( request->type == dhtt_msgs::srv::ModifyRequest::Request::REMOVE )
		{
			{
				std::lock_guard<std::mutex> guard(this->modify_mut);

				add_modify(request->to_modify);

				for (auto iter = request->to_modify.begin(); iter != request->to_modify.end(); iter++)
				{
					if ( not force and not this->can_remove(*iter) )
					{
						RCLCPP_ERROR(this->get_logger(), "Cannot remove node %s due to precondition violation later in the tree. Try again or force removal!");
						remove_modify(request->to_modify);

						return;
					}

					response->error_msg = this->remove_node(response, *iter);
					response->success = not strcmp(response->error_msg.c_str(), "");

					// exit early if modification fails
					if ( not response->success )
					{
						RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());
						remove_modify(request->to_modify);

						return;
					}
				}

				remove_modify(request->to_modify);
			}

			int id = this->start_maintain("ROOT_0");
			this->wait_for_maintain(id);

			return;
		}

		if ( request->type == dhtt_msgs::srv::ModifyRequest::Request::MUTATE or request->type == dhtt_msgs::srv::ModifyRequest::Request::PARAM_UPDATE )
		{
			{
				std::lock_guard<std::mutex> guard(this->modify_mut);

				add_modify(request->to_modify);

				for (auto iter = request->to_modify.begin(); iter != request->to_modify.end(); iter++)
				{

					dhtt_msgs::msg::Subtree tmp = this->fetch_subtree_by_name(*iter);

					if ( tmp.tree_status == -1)
					{
						response->error_msg = "Node with name " + (*iter) + " not found. Returning in error!";
						response->success = false;

						remove_modify(request->to_modify);

						return;
					}

					if ( request->type == dhtt_msgs::srv::ModifyRequest::Request::MUTATE )
					{
						if ( not force and not this->can_mutate(*iter, request->mutate_type) )
						{
							RCLCPP_ERROR(this->get_logger(), "Cannot mutate [%s] to requested type [%s] without violation occuring. Returning in error!", (*iter).c_str(), request->mutate_type.c_str());

							response->success = false;
							response->error_msg = "Cannot mutate to requested type without violation occuring. Returning in error!";

							remove_modify(request->to_modify);

							return;
						}

						RCLCPP_INFO(this->get_logger(), "Mutating node [%s] to type %s", (*iter).c_str(), request->mutate_type.c_str());
					}
					else 
						RCLCPP_INFO(this->get_logger(), "Changing params of node [%s]", (*iter).c_str());

					this->node_map[(*iter)]->modify(request, response);

					// response->error_msg = res->error_msg;
					response->success = not strcmp(response->error_msg.c_str(), "");

					// RCLCPP_FATAL(this->get_logger(), "Done modifying [%s]", (*iter).c_str());

					// exit early if modification fails
					if ( not response->success )
					{
						RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

						remove_modify(request->to_modify);

						return;
					}
				}

				remove_modify(request->to_modify);
			}

			int id = this->start_maintain("ROOT_0");
			this->wait_for_maintain(id);

			return;
		}

		// currently skipping other modification types
		response->error_msg = "Unknown request type " + std::to_string(request->type) + " chosen. Returning in error.";
		response->success = false;

		RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

		return; 
	}

	void MainServer::control_callback( const std::shared_ptr<dhtt_msgs::srv::ControlRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ControlRequest::Response> response )
	{

		RCLCPP_INFO(this->get_logger(), "--- Control request recieved!");
		this->publish_root_status();

		// save tree
		if ( request->type == dhtt_msgs::srv::ControlRequest::Request::SAVE )
		{
			RCLCPP_WARN(this->get_logger(), "Request to save tree received!");

			response->error_msg = this->save_tree(request->file_name, request->file_path);
			response->success = not strcmp(response->error_msg.c_str(), "");

			return;
		}

		// start tree
		if ( request->type == dhtt_msgs::srv::ControlRequest::Request::START )
		{
			RCLCPP_WARN(this->get_logger(), "Request to start task execution received!");

			response->error_msg = this->start_tree();
			response->success = not strcmp(response->error_msg.c_str(), "");

			return;
		}

		// reset
		if ( request->type == dhtt_msgs::srv::ControlRequest::Request::RESET )
		{
			RCLCPP_INFO(this->get_logger(), "Reset request received!");

			while ( (int) this->being_modified.size() > 0 and rclcpp::ok() );

			if ( not rclcpp::ok() )
			{
				response->error_msg = "ROS died so tree was technically reset!";

				response->success = not strcmp(response->error_msg.c_str(), "");
				return;
			}

			response->error_msg = this->reset_tree();

			response->success = not strcmp(response->error_msg.c_str(), "");

			return;
		}

		if (request->type == dhtt_msgs::srv::ControlRequest::Request::STOP )
		{
			RCLCPP_INFO(this->get_logger(), "Interrupt request received!");

			std::shared_ptr<dhtt_msgs::srv::InternalControlRequest::Request> rq = std::make_shared<dhtt_msgs::srv::InternalControlRequest::Request>();
			rq->control_code = dhtt_msgs::srv::InternalControlRequest::Request::GRACEFULSTOP;

			{
				using namespace std::chrono_literals;

				if ( not this->internal_control_client->wait_for_service(1s))
				{
					response->error_msg = "Unable to reach interal control server. Returning in error.";
					response->success = false;

					RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

					return;
				}
			}

			// this is somewhat dangerous as the request can fail it just shouldn't fail
			auto result = this->internal_control_client->async_send_request(rq);

			response->error_msg = "";
			response->success = true;

			return;
		}
	}
	
	void MainServer::fetch_callback( const std::shared_ptr<dhtt_msgs::srv::FetchRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> response )
	{

		RCLCPP_INFO(this->get_logger(), "--- Fetch request recieved!");
		this->publish_root_status();

		// wait for the maintenance pipeline to finish before giving a response
		this->wait_for_maintain_all();

		if ( request->return_full_subtree )
		{
			// response->found_subtrees.push_back(this->construct_subtree_from_node_iter(this->node_list.tree_nodes.begin()));
			response->found_subtrees.push_back(this->node_list);
			response->success = true;

			return;
		}

		/// check the optional args in the order they are listed in the srv file

		// find subtrees based on some common name they share
		if ( strcmp(request->common_name.c_str(), "") )
		{
			response->found_subtrees = this->fetch_subtrees_by_common_name(request->common_name);

			// no sub trees were found
			if ((int)response->found_subtrees.size() == 0)
			{
				response->success = false;

				response->error_msg = "Failed to find subtrees with name: " + request->common_name;
				
				RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

				return;
			}

			response->success = true;

			return;
		}

		// find a specific subtree based on some given name
		if ( strcmp(request->node_name.c_str(), "") )
		{
			response->found_subtrees.push_back(this->fetch_subtree_by_name(request->node_name));

			if (response->found_subtrees[0].tree_status == -1)
			{
				response->success = false;

				response->error_msg = "Failed to find subtree with name: " + request->node_name;
				
				RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

				return;
			}

			response->success = true;

			return;
		}

		if ( request->node_type > 0 and request->node_type < (dhtt_msgs::srv::FetchRequest::Request::BEHAVIOR + 1) )
		{
			response->found_subtrees = this->fetch_subtrees_by_type(request->node_type);

			if ((int)response->found_subtrees.size() == 0)
			{
				response->success = false;

				response->error_msg = "Failed to find subtrees with type: " + request->node_type;
				
				RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

				return;
			}

			response->success = true;

			return;
		}

		response->success = false;
		response->error_msg = "Malformed request... return_full_subtree is false, and no arguments were given to search the tree.";
				
		RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

		return;
	}

	void MainServer::history_callback( const std::shared_ptr<dhtt_msgs::srv::HistoryRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::HistoryRequest::Response> response )
	{
		(void) request;
		this->publish_root_status();

		std::vector<std::string> list_cast { std::begin(this->history), std::end(this->history) };

		response->node_history = list_cast;
	}

	void MainServer::resource_callback( const std::shared_ptr<dhtt_msgs::srv::ResourceRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ResourceRequest::Response> response )
	{
		RCLCPP_INFO(this->get_logger(), "--- Resource request recieved!");

		std::lock_guard<std::mutex> resource_update_lock(*(this->resource_update_mut_ptr));

		// pull the resource lists
		auto resource_names = this->global_com->get_parameter_sync("dhtt_resources.names").as_string_array();
		auto resource_types = this->global_com->get_parameter_sync("dhtt_resources.types").as_integer_array();
		auto resource_channels = this->global_com->get_parameter_sync("dhtt_resources.channels").as_integer_array();
		auto resource_locks = this->global_com->get_parameter_sync("dhtt_resources.locks").as_bool_array();
		auto resource_owners = this->global_com->get_parameter_sync("dhtt_resources.owners").as_integer_array();

		if ( request->type == dhtt_msgs::srv::ResourceRequest::Request::ADD )
		{
			response->success = false;

			// append the resources as long as they do not duplicate members of our list
			for ( auto resource : request->to_modify.resource_state )
			{
				if ( std::find(resource_names.begin(), resource_names.end(), resource.name) == resource_names.end() )
				{
					resource_names.push_back( resource.name );
					resource_types.push_back( resource.type );
					resource_channels.push_back( resource.channel );
					resource_locks.push_back( resource.locked );
					resource_owners.push_back( resource.owners );

					response->success = true;
				}
			}

			// push the lists back to the param server
			this->global_com->set_parameters_sync({rclcpp::Parameter("dhtt_resources.names", resource_names), 
												rclcpp::Parameter("dhtt_resources.types", resource_types),
												rclcpp::Parameter("dhtt_resources.channels", resource_channels),
												rclcpp::Parameter("dhtt_resources.locks", resource_locks),
												rclcpp::Parameter("dhtt_resources.owners", resource_owners)});

			response->server_resources = dhtt_utils::vector_to_msg(resource_names, resource_types, resource_channels, resource_locks, resource_owners);

			RCLCPP_INFO(this->get_logger(), "--- Resource Request Resolved");
			return;
		}
		else if ( request->type == dhtt_msgs::srv::ResourceRequest::Request::REMOVE )
		{
			response->success = false;

			// find all of the indices that need to be removed
			std::stack<int> to_remove;

			for ( auto resource : request->to_modify.resource_state )
			{
				auto iter = std::find(resource_names.begin(), resource_names.end(), resource.name);

				if ( iter != resource_names.end() )
					to_remove.push(std::distance(resource_names.begin(), iter));
			}

			// now remove them in reverse order
			int index = -1;
			
			while( (int) to_remove.size() > 0 )
			{
				index = to_remove.top();

				resource_names.erase(resource_names.begin() + index);
				resource_types.erase(resource_types.begin() + index);
				resource_channels.erase(resource_channels.begin() + index);
				resource_locks.erase(resource_locks.begin() + index);
				resource_owners.erase(resource_owners.begin() + index);

				response->success = true; // as long as a single resource was removed
				to_remove.pop();
			}

			// push the lists back to the param server
			this->global_com->set_parameters_sync({rclcpp::Parameter("dhtt_resources.names", resource_names), 
												rclcpp::Parameter("dhtt_resources.types", resource_types),
												rclcpp::Parameter("dhtt_resources.channels", resource_channels),
												rclcpp::Parameter("dhtt_resources.locks", resource_locks),
												rclcpp::Parameter("dhtt_resources.owners", resource_owners)});

			response->server_resources = dhtt_utils::vector_to_msg(resource_names, resource_types, resource_channels, resource_locks, resource_owners);
			RCLCPP_INFO(this->get_logger(), "--- Resource Request Resolved");
			return;
		}
		else if ( request->type == dhtt_msgs::srv::ResourceRequest::Request::GET )
		{
			response->server_resources = dhtt_utils::vector_to_msg(resource_names, resource_types, resource_channels, resource_locks, resource_owners);
			response->success = true;

			RCLCPP_INFO(this->get_logger(), "--- Resource Request Resolved");
			return;
		}

		response->success = false;

		RCLCPP_INFO(this->get_logger(), "--- Resource Request Resolved");
		return;
	}

	// modify helpers
	// I should also think about the node number here
	std::string MainServer::add_node( std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string parent_name, dhtt_msgs::msg::Node to_add, bool force, int index)
	{
		auto is_parent = [&]( dhtt_msgs::msg::Node check ) { return not strcmp(parent_name.c_str(), check.node_name.c_str()); };

		std::vector<dhtt_msgs::msg::Node>::iterator found_parent = std::find_if( this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), is_parent);

		if ( ( to_add.type < dhtt_msgs::msg::Node::AND ) or ( to_add.type > dhtt_msgs::msg::Node::BEHAVIOR ) )
		{
			return "Can\'t add a node with type " + std::to_string(to_add.type) + ". Returning in error.";
		}

		if ( found_parent == this->node_list.tree_nodes.end() )
		{
			return "Parent " + parent_name + " not found in tree. Returning in error.";
		}

		if ( not node_map[parent_name]->logic->can_add_child() )
		{
			return "Parent cannot be a BEHAVIOR type node. Returning in error!";
		}

		if ( index > (int) this->node_map[parent_name]->get_child_names().size() )
		{
			return "Cannot add node to parent at given index: index is out of bounds!";
		}

		if (to_add.type != dhtt_msgs::msg::Node::BEHAVIOR and
			to_add.plugin_name != MainServer::NODE_TYPE_TO_PLUGIN.at(to_add.type))
		{
			return "Node plugin name " + to_add.plugin_name + " does not match node type " +
				   std::to_string(to_add.type);
		}

		// add to our list of nodes (this should just modify the local copy)
		to_add.node_name += "_" + std::to_string(this->total_nodes_added);

		if (this->verbose)
			RCLCPP_INFO(this->get_logger(), "\tAdding node %s to the tree with parent %s.", to_add.node_name.c_str(), (*found_parent).node_name.c_str());

		//// cool thing I learned while debugging: push_back seems to cause problems with iterators that are found before it is used. So instead change everything and then push back is the solution!
		// this should already be set but in the case of the topmost node in a file it won't be for instance
		to_add.parent_name = std::string((*found_parent).node_name.c_str());
		to_add.parent = std::distance(this->node_list.tree_nodes.begin(), found_parent);
		to_add.node_status.state = dhtt_msgs::msg::NodeStatus::WAITING;
		
		// ensure that the goitr type is added if it exists
		std::string goitr_type = "";
		std::string potential_type = "dhtt_plugins::EfficiencyPotential";

		if ( strcmp(to_add.goitr_name.c_str(), ""))
			goitr_type = to_add.goitr_name;

		if ( strcmp(to_add.potential_type.c_str(), "" ) )
			potential_type = to_add.potential_type;

		// create a physical node from the message and add to physical list
		this->node_map[to_add.node_name] = std::make_shared<dhtt::Node>(this->global_com, to_add.node_name, to_add.plugin_name, to_add.params, parent_name, "dhtt_plugins::PtrBranchSocket", goitr_type, potential_type);

		if (this->node_map[to_add.node_name]->loaded_successfully() == false)
		{
			std::string err = this->node_map[to_add.node_name]->get_error_msg();

			this->node_map.erase(to_add.node_name);

			return err;
		}

		std::shared_ptr<dhtt_utils::PredicateConjunction> tmp1 = std::make_shared<dhtt_utils::PredicateConjunction>();
		std::shared_ptr<dhtt_utils::PredicateConjunction> tmp2 = std::make_shared<dhtt_utils::PredicateConjunction>();

		*tmp1 = this->node_map[to_add.node_name]->logic->get_preconditions();
		*tmp2 = this->node_map[to_add.node_name]->logic->get_postconditions(); 

		to_add.preconditions = dhtt_utils::to_string(tmp1);
		to_add.postconditions = dhtt_utils::to_string(tmp2);

		if ( not force and not this->can_add(to_add, index) )
		{
			this->node_map.erase(to_add.node_name);

			// sometimes get_error_msg() is empty, which is misinterpreted later as success.
			return "Precondition violation found in subsequent behaviors. Force not set and failed pre/postconditions check";
		}

		// this->spinner_cp->add_node(this->node_map[to_add.node_name]);
		// this->node_map[to_add.node_name]->register_with_parent(index);
		this->node_map[to_add.node_name]->register_servers();

		if (this->node_map[to_add.node_name]->loaded_successfully() == false)
		{
			std::string err = this->node_map[to_add.node_name]->get_error_msg();

			this->node_map.erase(to_add.node_name);

			return err;
		}

		this->node_map[to_add.parent_name]->add_child(this->node_map[to_add.node_name]->get_socket_ptr(), to_add.node_name, "dhtt_plugins::PtrBranchPlug", index);

		// std::string condition_topic = std::string(TREE_PREFIX) + "/" + to_add.node_name + CONDITION_POSTFIX;
		// this->maintenance_client_ptrs[to_add.node_name] = rclcpp_action::create_client<dhtt_msgs::action::Condition>(this, condition_topic, this->conc_group);

		// update the parent
		auto index_iter = (index == -1)? (*found_parent).children.end() : (*found_parent).children.begin() + index;
		auto name_index_iter = (index == -1)? (*found_parent).child_name.end() : (*found_parent).child_name.begin() + index;

		(*found_parent).children.insert( index_iter, (int) this->node_list.tree_nodes.size() );
		(*found_parent).child_name.insert( name_index_iter, to_add.node_name );

		this->node_list.tree_nodes.push_back(to_add);

		// add this node_name to the list of added nodes
		response->added_nodes.push_back(to_add.node_name);

		// finally add to the counter so that we continue to get unique names (up until 2^64 nodes are added of course)
		this->total_nodes_added++;

		return "";
	}

	std::string MainServer::add_nodes_from_file(std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string parent_name, std::string file_name, std::vector<std::string> file_args, bool force )
	{
		std::string cur_parent;
		bool first = true;

		auto is_parent = [&]( dhtt_msgs::msg::Node check ) { return not strcmp(parent_name.c_str(), check.node_name.c_str()); };

		// TODO remove this when done with cooking_zoo
		this->cooking_zoo_counter++;

		// making local add_node to avoid extra work that the add_node public function has to do for error checking etc. Returns the index to the added node
		auto local_add_node = [&](int parent_index, dhtt_msgs::msg::Node to_add, bool force, int index)
		{
			if ( ( to_add.type < dhtt_msgs::msg::Node::AND ) or ( to_add.type > dhtt_msgs::msg::Node::BEHAVIOR ) )
			{
				return -1;
			}

			if ( not node_map[parent_name]->logic->can_add_child() )
			{
				return -1;
			}

			auto found_parent = this->node_list.tree_nodes.begin() + parent_index;
			
			// add to our list of nodes (this should just modify the local copy)
			// to_add.node_name += "_" + std::to_string(this->total_nodes_added);
			
			if (this->verbose)
				RCLCPP_INFO(this->get_logger(), "\tAdding node %s to the tree with parent %s.", to_add.node_name.c_str(), (*found_parent).node_name.c_str());

			//// cool thing I learned while debugging: push_back seems to cause problems with iterators that are found before it is used. So instead change everything and then push back is the solution!
			// this should already be set but in the case of the topmost node in a file it won't be for instance
			to_add.parent_name = std::string((*found_parent).node_name.c_str());
			to_add.parent = std::distance(this->node_list.tree_nodes.begin(), found_parent);
			to_add.node_status.state = dhtt_msgs::msg::NodeStatus::WAITING;
			
			// ensure that the goitr type is added if it exists
			std::string goitr_type = "";
			std::string potential_type = "dhtt_plugins::EfficiencyPotential";

			if ( strcmp(to_add.goitr_name.c_str(), ""))
				goitr_type = to_add.goitr_name;

			if ( strcmp(to_add.potential_type.c_str(), "" ) )
				potential_type = to_add.potential_type;

			// create a physical node from the message and add to physical list
			this->node_map[to_add.node_name] = std::make_shared<dhtt::Node>(this->global_com, to_add.node_name, to_add.plugin_name, to_add.params, parent_name, "dhtt_plugins::PtrBranchSocket", goitr_type, potential_type);

			if (this->node_map[to_add.node_name]->loaded_successfully() == false)
			{
				std::string err = this->node_map[to_add.node_name]->get_error_msg();

				this->node_map.erase(to_add.node_name);

				return -1;
			}

			std::shared_ptr<dhtt_utils::PredicateConjunction> tmp1 = std::make_shared<dhtt_utils::PredicateConjunction>();
			std::shared_ptr<dhtt_utils::PredicateConjunction> tmp2 = std::make_shared<dhtt_utils::PredicateConjunction>();

			*tmp1 = this->node_map[to_add.node_name]->logic->get_preconditions();
			*tmp2 = this->node_map[to_add.node_name]->logic->get_postconditions(); 

			to_add.preconditions = dhtt_utils::to_string(tmp1);
			to_add.postconditions = dhtt_utils::to_string(tmp2);

			if ( not force and not this->can_add(to_add, index) )
			{
				this->node_map.erase(to_add.node_name);

				// sometimes get_error_msg() is empty, which is misinterpreted later as success.
				return -1;
			}

			// no longer relevant because they are not rclcpp:Node's
			// this->spinner_cp->add_node(this->node_map[to_add.node_name]);
			// this->node_map[to_add.node_name]->register_with_parent(index);
			this->node_map[to_add.node_name]->register_servers();

			if (this->node_map[to_add.node_name]->loaded_successfully() == false)
			{
				this->node_map.erase(to_add.node_name);

				return -1;
			}

			this->node_map[to_add.parent_name]->add_child(this->node_map[to_add.node_name]->get_socket_ptr(), to_add.node_name, "dhtt_plugins::PtrBranchPlug", index);

			// update the parent
			auto index_iter = (index == -1)? (*found_parent).children.end() : (*found_parent).children.begin() + index;
			auto name_index_iter = (index == -1)? (*found_parent).child_name.end() : (*found_parent).child_name.begin() + index;

			(*found_parent).children.insert( index_iter, (int) this->node_list.tree_nodes.size() );
			(*found_parent).child_name.insert( name_index_iter, to_add.node_name );

			this->node_list.tree_nodes.push_back(to_add);

			// add this node_name to the list of added nodes
			response->added_nodes.push_back(to_add.node_name);

			// finally add to the counter so that we continue to get unique names (up until 2^64 nodes in which case we should have given up long ago)
			this->total_nodes_added++;

			return (int) this->node_list.tree_nodes.size() - 1;
		};

		std::function<std::string(dhtt_msgs::msg::Subtree&, int, int)> add_post_order = [&]( dhtt_msgs::msg::Subtree& to_add, int current, int parent_index )
		{
			// RCLCPP_ERROR(this->get_logger(), "Add post order %s", to_add.tree_nodes[current].node_name.c_str());

			// TODO remove this when done with cooking_zoo
			for (auto& param : to_add.tree_nodes[current].params)
			{
				const auto index = param.find('#');

				if ( index != std::string::npos )
				{
					std::string to_rep = std::to_string(this->cooking_zoo_counter);

					param.replace(index, 1, "");
					param.insert(index, to_rep);
				}
			}

			// add node to the tree
			cur_parent = to_add.tree_nodes[current].parent_name;

			// copy and then delete the children list of the node to avoid duplication
			std::vector<int> children_cp = to_add.tree_nodes[current].children;

			to_add.tree_nodes[current].child_name.clear();
			to_add.tree_nodes[current].children.clear();

			int index = 0;

			if (first)
			{
				index = -1;
				first = false;
			}

			// err 
			int n_index = local_add_node(parent_index, to_add.tree_nodes[current], force, index); //(res, (*found_parent).node_name, to_add.tree_nodes[current], force, index);

			if ( n_index == -1 )
				return "Could not add node " + to_add.tree_nodes[current].node_name + " returning in error.";

			// update own name 
			std::string new_name = this->node_list.tree_nodes.back().node_name;
			response->added_nodes.push_back(new_name);

			// update children's version of parent name and add children in reverse order
			for ( auto riter = children_cp.rbegin() ; riter != children_cp.rend() ; riter++ )
			{
				to_add.tree_nodes[*riter].parent_name = new_name;

				std::string err_msg = add_post_order(to_add, *riter, n_index);

				if ( strcmp(err_msg.c_str(), "") )
					return err_msg;
			}

			// return empty if successful
			return std::string();
		};

		{ // placement here should be in the modify_cb where it was, but if we do it that way we don't have the lock for the status callback.
			// therefore I grab the mutex here which should be equivalent, just less clean.
			std::lock_guard<std::mutex> guard(this->modify_mut);

			// first load all of the yaml nodes into the message format
			if (this->verbose)
				RCLCPP_INFO(this->get_logger(), "Adding subtree loaded from file %s", file_name.c_str());

			dhtt_msgs::msg::Subtree nodes_to_add;

			std::string err_msg = this->construct_subtree_from_yaml(nodes_to_add, file_name, file_args);

			if ( strcmp(err_msg.c_str(), "") )
				return err_msg;

			nodes_to_add.tree_nodes[0].parent_name = parent_name;

			int parent_index = std::distance ( this->node_list.tree_nodes.begin(), std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), is_parent));

			// add nodes post order
			std::string error_msg = add_post_order(nodes_to_add, 0, parent_index);

			// make sure to pass the error through if anything fails
			if ( strcmp(error_msg.c_str(), "") )
			{// additional cleanup necessary?
				this->remove_node(std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>(), nodes_to_add.tree_nodes[0].node_name);

				return error_msg;
			}
			
			this->set_changed_up_tree(this->node_list.tree_nodes.end() - 1);
		}

		// then maintain the pre and post conditions
		int id = this->start_maintain("ROOT_0");

		this->wait_for_maintain(id);

		return "";
	}

	std::string MainServer::remove_node( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string to_remove )
	{
		dhtt_msgs::msg::Subtree subtree_to_remove = this->fetch_subtree_by_name(to_remove);

		// lambda and helper variable for finding a node with look_for as a name in a list of nodes
		std::string look_for;
		auto is_named = [&]( dhtt_msgs::msg::Node check ) { return not strcmp(check.node_name.c_str(), look_for.c_str()); };
		auto is_str = [&]( std::string check ) { return not strcmp(check.c_str(), look_for.c_str()) ; };

		if ( subtree_to_remove.tree_status == -1 )
			return "Failed to find subtree with name: " + to_remove;

		// removing the root node is currently not allowed so just return in error
		if ( not strcmp( subtree_to_remove.tree_nodes[0].node_name.c_str(), this->node_list.tree_nodes[0].node_name.c_str() ) )
			return "Cannot remove root node of the tree. Returning in error.";

		if (this->verbose)
			RCLCPP_INFO(this->get_logger(), "Removing subtree with root %s", subtree_to_remove.tree_nodes[0].node_name.c_str());
		
		// this assumes that parents are always before their children in the tree_nodes list bc otherwise the indices would have to be adjusted every time a node is removed
		for (std::vector<dhtt_msgs::msg::Node>::reverse_iterator node_iter = subtree_to_remove.tree_nodes.rbegin() ; node_iter != subtree_to_remove.tree_nodes.rend() ; node_iter++ )
		{
			look_for = (*node_iter).node_name;

			auto found_node_iter = std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), is_named);

			// this should never evaluate to true as we already found the nodes in the tree before. (could happen if the user is trying to double remove nodes)
			if ( found_node_iter == this->node_list.tree_nodes.end())
				return "Could not find node " + look_for + " in internal list of nodes. Returning in error.";

			std::string parent_name = (*found_node_iter).parent_name;

			int parent_index = (*found_node_iter).parent;
			int remove_index = std::distance(this->node_list.tree_nodes.begin(), found_node_iter);

			if (this->verbose)
				RCLCPP_INFO(this->get_logger(), "\tRemoving node %s from parent %s", look_for.c_str(), this->node_list.tree_nodes[parent_index].node_name.c_str() );

			std::remove(this->node_list.tree_nodes[parent_index].children.begin(), this->node_list.tree_nodes[parent_index].children.end(), remove_index);
			std::remove_if(this->node_list.tree_nodes[parent_index].child_name.begin(), this->node_list.tree_nodes[parent_index].child_name.end(), is_str);

			// make sure the vectors are resized otherwise there will be an empty element left over
			this->node_list.tree_nodes[parent_index].children.resize(this->node_list.tree_nodes[parent_index].children.size() - 1);
			this->node_list.tree_nodes[parent_index].child_name.resize(this->node_list.tree_nodes[parent_index].child_name.size() - 1);

			this->node_list.tree_nodes.erase(found_node_iter);

			// remove the maintenance client as well
			this->maintenance_client_ptrs.erase(to_remove);

			response->removed_nodes.push_back(look_for);

			// now remove the node and it's reference from the parent from the real node map, also remove from the spinner
			this->node_map[parent_name]->remove_child(look_for);

			// this->spinner_cp->remove_node(this->node_map[look_for]);
			this->node_map.erase(look_for);
		}

		if ( (int) this->node_list.tree_nodes.size() == 0 )
			return "";

		// instead they just have to be maintained after the subtree is removed
		this->maintain_local_subtree();

		int id = this->start_maintain("ROOT_0");
		this->wait_for_maintain(id);

		return "";
	}
	
	std::string MainServer::change_params( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request )
	{
		(void) request;

		return "";
	}

	dhtt_utils::PredicateConjunction MainServer::get_postconditions(int subtree_index)
	{
		return dhtt_utils::convert_to_struct(this->node_list.tree_nodes[subtree_index].postconditions);
	}

	dhtt_utils::PredicateConjunction MainServer::get_preconditions(int subtree_index)
	{
		return dhtt_utils::convert_to_struct(this->node_list.tree_nodes[subtree_index].preconditions);
	}

	dhtt_utils::PredicateConjunction MainServer::collect_previous_postconditions(std::string parent_name, int child_index)
	{
		// basic setup
		dhtt_utils::PredicateConjunction to_ret;
		to_ret.logical_operator = dhtt_utils::LOGICAL_AND;

		auto find_name = [&]( dhtt_msgs::msg::Node to_check ) { return not strcmp(parent_name.c_str(), to_check.node_name.c_str()); };

		int iter = std::distance(this->node_list.tree_nodes.begin(), std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), find_name));

		int prev_index = this->node_list.tree_nodes[iter].children[child_index];

		auto find_index = [&]( int to_check ) { return prev_index == to_check; };

		std::stack<dhtt_utils::PredicateConjunction> pred_stack;
		bool first = true;

		// all the way up the tree (0 is always the root node)
		while ( iter > 0 ) 
		{
			// if it's a then node
			if ( this->node_list.tree_nodes[iter].type == dhtt_msgs::msg::Node::THEN )
			{

				// take all the subsequent children
				dhtt_msgs::msg::Node ancestor = this->node_list.tree_nodes[iter];
				int my_child_index = std::distance(ancestor.children.begin(), std::find_if(ancestor.children.begin(), ancestor.children.end(), find_index));
				// RCLCPP_WARN(this->get_logger(), "Found Ancestor %s", ancestor.node_name.c_str());

				if ( not first )
					my_child_index -= 1;
				else 
					first = false;

				// push the left behaviors onto the stack
				for ( int i = my_child_index; i >= 0; i-- )
				{
					// RCLCPP_ERROR(this->get_logger(), "Pushing child [%s] onto the stack", this->node_list.tree_nodes[ancestor.children[i]].node_name.c_str());
					pred_stack.push(this->get_postconditions(ancestor.children[i]));
				}

			} 

			// continue up the tree
			prev_index = iter;
			iter = this->node_list.tree_nodes[iter].parent;
		}

		while ( not pred_stack.empty() )
		{
			dhtt_utils::PredicateConjunction popped_pred = pred_stack.top();

			dhtt_utils::append_predicate_conjunction(to_ret, popped_pred);
			dhtt_utils::remove_predicate_partial_duplicates(to_ret);
			// RCLCPP_WARN(this->get_logger(), "%s -----------> %s", dhtt_utils::to_string(popped_pred).c_str(),dhtt_utils::to_string(to_ret).c_str() );

			pred_stack.pop();
		}


		return to_ret;
	}

	// need to change the node child index that we are looking for as we go up the tree
	std::vector<int> MainServer::get_next_behaviors(std::string parent_name, int child_index, int offset)
	{
		// basic setup
		std::vector<int> to_ret;

		auto find_name = [&]( dhtt_msgs::msg::Node to_check ) { return not strcmp(parent_name.c_str(), to_check.node_name.c_str()); };

		int iter = std::distance(this->node_list.tree_nodes.begin(), std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), find_name));
		int prev_index = this->node_list.tree_nodes[iter].children[child_index + offset];

		auto find_index = [&]( int to_check ) { return prev_index == to_check; };

		bool first = true;

		// all the way up the tree (0 is always the root node)
		while ( iter > 0 ) 
		{
			// if it's a then node
			if ( this->node_list.tree_nodes[iter].type == dhtt_msgs::msg::Node::THEN )
			{

				// take all the subsequent children
				dhtt_msgs::msg::Node ancestor = this->node_list.tree_nodes[iter];
				int my_child_index = std::distance(ancestor.children.begin(), std::find_if(ancestor.children.begin(), ancestor.children.end(), find_index));

				if ( not first )
					my_child_index += offset;
				else 
					first = false;

				for ( int i = my_child_index; i < (int) ancestor.children.size(); i++ )
					to_ret.push_back(ancestor.children[i]);
			} 

			// continue up the tree
			prev_index = iter;
			iter = this->node_list.tree_nodes[iter].parent;
		}

		return to_ret;
	}

	bool MainServer::can_remove(std::string node_name)
	{
		auto name_match = [&]( dhtt_msgs::msg::Node check ) { return not strcmp(node_name.c_str(), check.node_name.c_str()); };

		int name_index = std::distance(this->node_list.tree_nodes.begin(), std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), name_match));

		// can't remove nonexistant nodes ( this should just be a backup check )
		if ( name_index >= (int) this->node_list.tree_nodes.size() )
			return false;

		// get position in parent list of children ( sorry for this monstrosity  )
		int index = -1;
		for ( index = 0; index < (int) this->node_list.tree_nodes[this->node_list.tree_nodes[name_index].parent].children.size(); index++ )
			if ( this->node_list.tree_nodes[this->node_list.tree_nodes[name_index].parent].children[index] == name_index )
				break;

		std::vector<int> forward_temporal_dependencies = this->get_next_behaviors(this->node_list.tree_nodes[name_index].parent_name, index, 1);
		auto postconditions = *dhtt_utils::negate_predicates(this->node_map[node_name]->logic->get_postconditions());

		int most_recent_parent = this->node_list.tree_nodes[name_index].parent;
		
		for ( int iter : forward_temporal_dependencies )
		{
			auto subsequent_preconditions = this->get_preconditions(iter);
			auto subsequent_postconditions = this->get_postconditions(iter);

			RCLCPP_DEBUG(this->get_logger(), "Check predicate %s for %s", dhtt_utils::to_string(postconditions).c_str(), this->node_list.tree_nodes[iter].node_name.c_str());

			if ( dhtt_utils::violates_predicates(postconditions, subsequent_preconditions) )
				return false; 

			if ( this->node_list.tree_nodes[iter].parent != most_recent_parent )
			{
				most_recent_parent = this->node_list.tree_nodes[iter].parent;
				dhtt_utils::append_predicate_conjunction(subsequent_postconditions, postconditions);
			
				postconditions = *dhtt_utils::conjunction_copy(subsequent_postconditions);// basically we want to follow the order of subsequent postconditions first
			}
			else
				dhtt_utils::append_predicate_conjunction(postconditions, subsequent_postconditions);

			dhtt_utils::remove_predicate_partial_duplicates(postconditions);
		}

		return true;
	}

	bool MainServer::can_add(dhtt_msgs::msg::Node to_add, int index)
	{
		if (to_add.parent_name == "ROOT_0")
			return true;

		// this all relies on the to_add node being in the internal representation of the tree
		// already ( not actually created tho )
		auto name_match = [&]( dhtt_msgs::msg::Node check ) { return not strcmp(to_add.parent_name.c_str(), check.node_name.c_str()); };

		int name_index = std::distance(this->node_list.tree_nodes.begin(), std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), name_match));

		auto preconditions = this->node_map[to_add.node_name]->logic->get_preconditions();
		auto postconditions = this->node_map[to_add.node_name]->logic->get_postconditions();

		// collect previous postconditions to get the world state before this new behavior
		auto prev_postconditions =
			this->collect_previous_postconditions(to_add.parent_name, index - 1);

		// check the new preconditions against the expected world state
		if ( dhtt_utils::violates_predicates(prev_postconditions, preconditions) )
			return false;

		std::vector<int> forward_temporal_dependencies = this->get_next_behaviors(to_add.parent_name, index);

		int most_recent_parent = name_index;
		
		for ( int iter : forward_temporal_dependencies )
		{
			auto subsequent_preconditions = this->get_preconditions(iter);
			auto subsequent_postconditions = this->get_postconditions(iter);

			RCLCPP_DEBUG(this->get_logger(), "Check predicate %s for %s", dhtt_utils::to_string(postconditions).c_str(), this->node_list.tree_nodes[iter].node_name.c_str());

			if ( dhtt_utils::violates_predicates(postconditions, subsequent_preconditions) )
				return false; 

			if ( this->node_list.tree_nodes[iter].parent != most_recent_parent )
			{
				most_recent_parent = this->node_list.tree_nodes[iter].parent;
				dhtt_utils::append_predicate_conjunction(subsequent_postconditions, postconditions);
			
				postconditions = *dhtt_utils::conjunction_copy(subsequent_postconditions);// basically we want to follow the order of subsequent postconditions first
			}
			else
				dhtt_utils::append_predicate_conjunction(postconditions, subsequent_postconditions);

			dhtt_utils::remove_predicate_partial_duplicates(postconditions);
		}

		return true;
	}

	bool MainServer::can_mutate(std::string node_name, std::string n_type)
	{
		/// check child order
		// get node iter to node name
		auto name_match = [&]( dhtt_msgs::msg::Node check ) { return not strcmp(node_name.c_str(), check.node_name.c_str()); };
		auto name_match_strings = [&]( std::string check ) { return not strcmp(check.c_str(), node_name.c_str()); };

		int name_index = std::distance(this->node_list.tree_nodes.begin(), std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), name_match));

		// get list of children
		std::vector<int> children = this->node_list.tree_nodes[name_index].children;
		
		// -> THEN
		if ( not strcmp( n_type.c_str() , "dhtt_plugins::ThenBehavior" ) )
		{

			// check pre/post condition relationships in owned_resources
			dhtt_utils::PredicateConjunction postconditions = this->get_postconditions(children.front());
			bool first = true;
		
			for ( int iter : children )
			{
				if ( first )
				{
					first = false;
					continue;
				}

				auto subsequent_preconditions = this->get_preconditions(iter);
				auto subsequent_postconditions = this->get_postconditions(iter);

				RCLCPP_DEBUG(this->get_logger(), "Check predicate %s for %s", dhtt_utils::to_string(postconditions).c_str(), this->node_list.tree_nodes[iter].node_name.c_str());

				if ( dhtt_utils::violates_predicates(postconditions, subsequent_preconditions) )
					return false; 

				dhtt_utils::append_predicate_conjunction(postconditions, subsequent_postconditions);
				dhtt_utils::remove_predicate_partial_duplicates(postconditions);
			}
		}

		/// check up the tree
		// mutate the postcondition child list such that it reflects the new change

		// first generate the map of child conditions with which to trick the node stuff below
		std::map<std::string, dhtt_msgs::action::Condition::Result::SharedPtr> trick_conditions;

		for ( int iter : children )
		{
			std::string name = this->node_list.tree_nodes[iter].node_name;
			dhtt_msgs::action::Condition::Result::SharedPtr conditions = std::make_shared<dhtt_msgs::action::Condition::Result>();

			conditions->success = true;
			conditions->error_msg = "";
			conditions->preconditions = dhtt_utils::to_string(this->get_preconditions(iter));
			conditions->postconditions = dhtt_utils::to_string(this->get_postconditions(iter));
			
			trick_conditions[name] = conditions;
		}

		// now make a dummy node of the new type to generate the postconditions list
		dhtt::Node dummy(this->global_com, "dummy", n_type, std::vector<std::string>(), "NONE");

		// if it doesn't load most likely the plugin does not exist or requires some parameters
		if ( not dummy.successful_load )
			return false;

		// now collect the postconditions by maintaining the conditions in the node logic
		dummy.child_conditions = trick_conditions;
		dummy.logic->maintain_conditions(&dummy);

		auto postconditions = dummy.logic->get_postconditions();

		int index = std::distance(this->node_list.tree_nodes[this->node_list.tree_nodes[name_index].parent].child_name.begin(),
						 std::find_if(this->node_list.tree_nodes[this->node_list.tree_nodes[name_index].parent].child_name.begin(), 
						 				this->node_list.tree_nodes[this->node_list.tree_nodes[name_index].parent].child_name.end(), name_match_strings));

		// RCLCPP_ERROR(this->get_logger(), "%d", index);

		// now check those postconditions going up the tree
		std::vector<int> forward_temporal_dependencies = this->get_next_behaviors(this->node_list.tree_nodes[this->node_list.tree_nodes[name_index].parent].node_name, index, 1);

		int most_recent_parent = name_index;
		
		for ( int iter : forward_temporal_dependencies )
		{
			auto subsequent_preconditions = this->get_preconditions(iter);
			auto subsequent_postconditions = this->get_postconditions(iter);

			RCLCPP_DEBUG(this->get_logger(), "Check predicate %s for %s", dhtt_utils::to_string(postconditions).c_str(), this->node_list.tree_nodes[iter].node_name.c_str());

			if ( dhtt_utils::violates_predicates(postconditions, subsequent_preconditions) )
			{
				// RCLCPP_INFO(this->get_logger(), "Hello paul im here too");
				return false; 
			}

			if ( this->node_list.tree_nodes[iter].parent != most_recent_parent )
			{
				most_recent_parent = this->node_list.tree_nodes[iter].parent;
				dhtt_utils::append_predicate_conjunction(subsequent_postconditions, postconditions);
			
				postconditions = *dhtt_utils::conjunction_copy(subsequent_postconditions);// basically we want to follow the order of subsequent postconditions first
			}
			else
				dhtt_utils::append_predicate_conjunction(postconditions, subsequent_postconditions);

			dhtt_utils::remove_predicate_partial_duplicates(postconditions);
		}

		return true;
	}

	// control helpers
	std::string MainServer::stop_tree( bool interrupt )
	{
		(void) interrupt;

		return "";
	}

	std::string MainServer::start_tree()
	{
		if ( this->running )
			return "Tree is already actively running. Returning in error.";
		{
			// there is no harm in starting a finished tree so just let the root know to try
			this->node_map["ROOT_0"]->update_status(dhtt_msgs::msg::NodeStatus::WAITING);
		}

		this->run_tree_thread = std::make_shared<std::thread>(&MainServer::run_tree, this);
		this->run_tree_thread->detach();

		return "";
	}

	std::string MainServer::save_tree(std::string file_name, std::string file_path)
	{
		struct stat path;

		std::string file;

		if ( stat( file_path.c_str(), &path ) == 0 )
		{
			file = file_name + file_path;
		}
		else
		{
			if ( strcmp(file_path.c_str(), "") )
				return "Specified file path does not exist, returning in error";

			file = dhtt_folder_path.native() + DEFAULT_SAVE_LOCATION + file_name;
		}

		// construct the yaml friendly version of the tree
		YAML::Node root_node;

		for ( auto const& iter : this->node_list.tree_nodes )
		{
			if ( not strcmp(iter.node_name.c_str(), "ROOT_0") )
				continue;

			// construct the current node
			YAML::Node current_node;

			current_node["type"] = iter.type;
			current_node["behavior_type"] = iter.plugin_name;
			current_node["goitr_type"] = iter.goitr_name;
			current_node["robot"] = 0; // for now we will always assume the one robot 
			current_node["parent"] = iter.parent_name;

			for ( auto param_iter = iter.params.begin() ; param_iter != iter.params.end() ; param_iter++ )
				current_node["params"].push_back( (*param_iter) );
			
			// add to the total
			root_node["NodeList"].push_back(iter.node_name);
			root_node["Nodes"][iter.node_name] = current_node; 
		}

		std::ofstream fout(file);
		fout << root_node;

		return "";
	}

	std::string MainServer::reset_tree()
	{

		if ( this->node_list.tree_status == dhtt_msgs::msg::NodeStatus::ACTIVE or this->node_list.tree_status == dhtt_msgs::msg::NodeStatus::WORKING or this->running )
		{
			return "Can\'t reset the tree while it is active. Returning in error.";
		}

		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> blank_rs = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		// remove the nodes after
		for ( std::string iter : this->node_list.tree_nodes[0].child_name )
			std::string blank = this->remove_node( blank_rs, iter );
			
		// reset all resources and params 
		this->node_map["ROOT_0"]->logic->release_all_resources();
		this->reset_param_server();

		this->history.clear();

		this->node_map["ROOT_0"]->update_status(dhtt_msgs::msg::NodeStatus::WAITING);

		return "";
	}

	void MainServer::reset_param_server()
	{
		// void the current robot resources
		this->global_com->set_parameters_sync({rclcpp::Parameter("dhtt_resources.names", std::vector<std::string>()), 
												rclcpp::Parameter("dhtt_resources.types", std::vector<int>()),
												rclcpp::Parameter("dhtt_resources.channels", std::vector<int>()),
												rclcpp::Parameter("dhtt_resources.locks", std::vector<bool>()),
												rclcpp::Parameter("dhtt_resources.owners", std::vector<int>())});

		// reload the robot resources from the yaml given initially
		this->node_map["ROOT_0"]->logic->initialize({});

		// now reset our marks ( TODO: REMOVE )
		this->global_com->set_parameter_sync(rclcpp::Parameter("world.marked_objects_taints", std::vector<std::string>()));
		this->global_com->set_parameter_sync(rclcpp::Parameter("world.marked_objects_types", std::vector<std::string>()));
		this->global_com->set_parameter_sync(rclcpp::Parameter("world.marked_objects_ids", std::vector<long int>()));
	}

	// fetch helpers
	std::vector<dhtt_msgs::msg::Subtree> MainServer::fetch_subtrees_by_common_name( std::string name )
	{
		std::vector<dhtt_msgs::msg::Subtree> to_ret;

		auto common_name = [&]( dhtt_msgs::msg::Node check ) { return check.node_name.find(name) != std::string::npos; };

		auto found_iter = std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), common_name);

		while (found_iter != this->node_list.tree_nodes.end())
		{
			to_ret.push_back(this->construct_subtree_from_node_iter(found_iter));

			found_iter++;

			found_iter = std::find_if(found_iter, this->node_list.tree_nodes.end(), common_name);
		}

		return to_ret;
	}

	std::vector<dhtt_msgs::msg::Subtree> MainServer::fetch_subtrees_by_type( int type )
	{
		std::vector<dhtt_msgs::msg::Subtree> to_ret;

		auto type_match = [&]( dhtt_msgs::msg::Node check ) { return check.type == type; };

		auto found_iter = std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), type_match);

		while (found_iter != this->node_list.tree_nodes.end())
		{
			to_ret.push_back(this->construct_subtree_from_node_iter(found_iter));

			found_iter++;

			found_iter = std::find_if(found_iter, this->node_list.tree_nodes.end(), type_match);
		}

		return to_ret;
	}

	dhtt_msgs::msg::Subtree MainServer::fetch_subtree_by_name( std::string name )
	{
		auto name_match = [&]( dhtt_msgs::msg::Node check ) { return not strcmp(name.c_str(), check.node_name.c_str()); };

		auto found_iter = std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), name_match);

		if (found_iter == this->node_list.tree_nodes.end())
		{
			dhtt_msgs::msg::Subtree failed_subtree;
			failed_subtree.tree_status = -1;

			return failed_subtree;
		}

		return this->construct_subtree_from_node_iter(found_iter);
	}

	void MainServer::maintainer_thread_cb()
	{
		dhtt_msgs::action::Condition::Result result;

		// condition from which to wake up this thread ( does this thread clean up? )
		auto check_queue_size = [&](){ return this->maintenance_queue.size() > 0 or this->end; };

		// useful example of using condition variables here: https://en.cppreference.com/w/cpp/thread/condition_variable
		while ( rclcpp::ok() )
		{
			// Create lock and then wait for the queue to have a size greater than 1
			std::unique_lock<std::mutex> lock(this->maintenance_mut);

			if ( (int) this->maintenance_queue.size() == 0 )
				maintenance_queue_condition.wait(lock, check_queue_size);
			else
				lock.lock();

			if ( this->end )
				return;

			// after wait the lock is acquired so take the first name in the queue and send the maintenance request
			std::string node_name = this->maintenance_queue.front();
			this->maintenance_queue.pop();

			RCLCPP_INFO(this->get_logger(), "Sending maintenance request to %s...", node_name.c_str());

			auto condition_goal = dhtt_msgs::action::Condition::Goal();
			condition_goal.type = dhtt_msgs::action::Condition::Goal::MAINTAIN;

			auto result = this->node_map[node_name]->combine_child_conditions(condition_goal);

			RCLCPP_INFO(this->get_logger(), "Received condition response from %s.", node_name.c_str());

			if ( not result.success )
				RCLCPP_ERROR(this->get_logger(), "Maintenance request for %s failed with error_msg: %s", node_name.c_str(), result.error_msg.c_str());

			// change finished id to the id that was just performed then return
			this->last_finished_id = this->maintenance_dict[node_name];
			this->maintenance_dict.erase(this->maintenance_dict.find(node_name));

			finished_maintenance_id_condition.notify_one();

			// unlock and then notify any waiting that the queue has shrunk by one
			lock.unlock();
		}
	}

	int MainServer::start_maintain(std::string node_name)
	{
		// grab the lock and add node_name to the queue
		{
			std::lock_guard<std::mutex> maintain_guard(this->maintenance_mut);

			this->maintenance_queue.push(node_name);
			this->maintenance_dict[node_name] = this->used_id;
			this->used_id++;

			this->maintenance_queue_condition.notify_one();
		}

		return this->maintenance_dict[node_name];
	}

	void MainServer::wait_for_maintain(int id)
	{
		auto maintain_finished = [&](){ return this->last_finished_id == id; };

		std::unique_lock<std::mutex> lock(this->maintenance_mut);

		// otherwise wait for the notification
		this->finished_maintenance_id_condition.wait(lock, maintain_finished);
	}

	void MainServer::wait_for_maintain_all()
	{
		while ( (int) this->maintenance_queue.size() > 0 and rclcpp::ok() );

		return;
	}


	// subscriber callbacks
	void MainServer::status_callback( const std::shared_ptr<dhtt_msgs::msg::Node> data )
	{
		// pick up the modify_mut lock so that nothing invalidates our iterator as we go
		std::lock_guard<std::mutex> guard(this->modify_mut);
		
		// unary lambda function for find_if which takes advantage of the scope inside this function. This function is not useful elsewhere
		auto same_name = [&](dhtt_msgs::msg::Node check) { return not strcmp(data->node_name.c_str(), check.node_name.c_str()); };

		auto found_node = std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), same_name);

		// node with matching name was not found
		if ( found_node == this->node_list.tree_nodes.end() )
			return;

		(*found_node).node_status.state = data->node_status.state;
		(*found_node).owned_resources = data->owned_resources;
		(*found_node).plugin_name = data->plugin_name;

		(*found_node).preconditions.clear();
		(*found_node).postconditions.clear();

		(*found_node).preconditions = data->preconditions;
		(*found_node).postconditions = data->postconditions;

		// RCLCPP_INFO(this->get_logger(), "%s: %s, %s", (*found_node).node_name.c_str(), (*found_node).preconditions.c_str(), (*found_node).postconditions.c_str());

		bool is_leaf = (*found_node).child_name.empty();

		// if the node is working and is not the head of the history make it the head of the history. only collect behaviors not tasks (so leaves)
		if ( data->node_status.state == dhtt_msgs::msg::NodeStatus::WORKING and ( ( (int) this->history.size() == 0 ) or strcmp ( this->history.back().c_str(), data->node_name.c_str() ) ) and is_leaf )
		{
			RCLCPP_WARN(this->get_logger(), "Adding node %s to history!!!!!", data->node_name.c_str());
			this->history.push_back((*data).node_name);
		}

		// make sure to publish root status
		if ( not strcmp(data->node_name.c_str(), "ROOT_0") )
			this->publish_root_status();
	}

	// generally helpful functions
	void MainServer::maintain_local_subtree()
	{
		// we are going to rebuild the tree indices because we know the parent, node, and child names. So first we'll clear all of those index members
		for ( std::vector<dhtt_msgs::msg::Node>::iterator node_iter = this->node_list.tree_nodes.begin() ; node_iter != this->node_list.tree_nodes.end() ; node_iter++ )
		{
			(*node_iter).parent = -1;
			(*node_iter).children = std::vector<int>((int) (*node_iter).child_name.size(), -1);
		}

		dhtt_msgs::msg::Node search_for;
		auto is_parent = [&]( dhtt_msgs::msg::Node check ){ return not strcmp(search_for.parent_name.c_str(), check.node_name.c_str()); };

		// iterate in reverse through the tree in order to update all parent and child positions
		for ( std::vector<dhtt_msgs::msg::Node>::reverse_iterator node_iter = this->node_list.tree_nodes.rbegin() ; node_iter != this->node_list.tree_nodes.rend() ; node_iter++ )
		{
			search_for = *node_iter;

			auto parent_iter = std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), is_parent);

			// this should only happen for the root node and otherwise is happening in error
			if ( parent_iter == this->node_list.tree_nodes.end() )
				continue;

			(*node_iter).parent = std::distance(this->node_list.tree_nodes.begin(), parent_iter);

			int child_index = -1;

			for (int i = 0; i < (int) ((*parent_iter).child_name.size()); i++)
			{
				if ( not strcmp((*parent_iter).child_name[i].c_str(), (*node_iter).node_name.c_str()) )
				{
					child_index = i;
					break;
				}
			}

			(*parent_iter).children[child_index] = this->node_list.tree_nodes.size() - std::distance(this->node_list.tree_nodes.rbegin(), node_iter) - 1;
		}

		// finally erase and then maintain the overall tree metrics
		this->node_list.tree_status = -1;
		this->node_list.max_tree_depth = -1;
		this->node_list.max_tree_width = -1;
		this->node_list.task_completion_percent = -1.0;

		this->fill_subtree_metrics(this->node_list);
	}

	void MainServer::fill_subtree_metrics( dhtt_msgs::msg::Subtree& to_fill )
	{
		//calculate the max depth and max width by doing a linear walk
		std::vector<int> depth_helper((int)to_fill.tree_nodes.size(), 0);
		std::vector<int> width_helper((int)to_fill.tree_nodes.size(), 0);
		int num_completed = 0;

		for (auto node_iter = to_fill.tree_nodes.begin(); node_iter != to_fill.tree_nodes.end(); node_iter++)
		{
			int cur_index = std::distance(to_fill.tree_nodes.begin(), node_iter);

			// update width helper
			width_helper[cur_index] = (int)((*node_iter).children.size());

			// check if the node is completed
			if ((*node_iter).node_status.state == dhtt_msgs::msg::NodeStatus::DONE)
				num_completed++;

			if (node_iter == to_fill.tree_nodes.begin())
				continue;

			// update the depth helper list
			depth_helper[cur_index] = depth_helper[(*node_iter).parent] + 1;
			
		}

		// calculate the metrics using the helpers
		to_fill.tree_status = (*to_fill.tree_nodes.begin()).node_status.state;
		to_fill.max_tree_depth = *std::max_element(depth_helper.begin(), depth_helper.end());
		to_fill.max_tree_width = *std::max_element(width_helper.begin(), width_helper.end());
		to_fill.task_completion_percent = num_completed / ((int) to_fill.tree_nodes.size());

		// for some reason we are getting error: "free() invalid pointer" after this functin returns
		// this only happens after removing any node from the tree and I'm currently not sure why

		return;
	}

	void MainServer::run_tree()
	{	
		// need to allow for interruptions soon but for now assume we run until the task finishes
		std::lock_guard<std::mutex> lock(this->running_mut);
		this->running = true;

		dhtt_msgs::action::Activation::Goal g;

		RCLCPP_INFO(this->get_logger(), "Activating the root node...");

		// make sure that the root node is not waiting on resources before running the tree
		this->node_map["ROOT_0"]->set_resource_status_updated(true);
		this->node_map["ROOT_0"]->activate(g);

		RCLCPP_INFO(this->get_logger(), "Tree finished running...");
		this->running = false;
	}
	
	void MainServer::tree_result_callback(const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Activation>::WrappedResult & result)
	{
		// empty for now, this is where we can get information about the tree once it finishes the task
		(void) result;


		// this->running_mut.unlock();
	}

	void MainServer::publish_root_status()
	{
		// RCLCPP_INFO(this->get_logger(), "Publishing root status!");

		dhtt_msgs::msg::NodeStatus to_pub;
		to_pub.state = this->node_list.tree_nodes[0].node_status.state;

		this->root_status_pub->publish(to_pub);
	}

	dhtt_msgs::msg::Node MainServer::get_active_behavior_node()
	{
		// unary lambda function for find_if which be annoying to have in outside namespace. This function is not useful elsewhere
		auto active_behavior = [](dhtt_msgs::msg::Node check) { return check.node_status.state == dhtt_msgs::msg::NodeStatus::ACTIVE and check.type == dhtt_msgs::msg::Node::BEHAVIOR; };

		auto found_node = std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), active_behavior);

		if ( found_node == this->node_list.tree_nodes.end() )
		{
			dhtt_msgs::msg::Node fail_node;

			fail_node.node_name = FAILED;

			return fail_node;
		}

		return *found_node;
	}

	std::vector<dhtt_msgs::msg::Node> MainServer::get_active_nodes()
	{
		std::vector<dhtt_msgs::msg::Node> to_ret;

		for (dhtt_msgs::msg::Node iter_node : this->node_list.tree_nodes)
			if (iter_node.node_status.state == dhtt_msgs::msg::NodeStatus::ACTIVE)
				to_ret.push_back(iter_node);

		return to_ret;
	}

	void MainServer::set_changed_up_tree(std::vector<dhtt_msgs::msg::Node>::iterator node)
	{
		if ( strcmp(node->parent_name.c_str(), "ROOT_0") )
		{
			auto parent = this->node_list.tree_nodes.begin() + node->parent;

			this->node_map[node->parent_name]->set_child_changed(node->node_name, true);
			this->node_map[node->node_name]->set_changed_flag(true);

			this->set_changed_up_tree(parent);
		}
	}

	// for this to work I have to maintain that the children of each node will be next to each other even if they are not next to their parent <-- I don't think this is true. I think as long as the parent and child indices are maintained it should be fine
	// then this maintains order, and also ensures that that is kept as well. I am concerned for the runtime of this algorithm (looks like O(n^2)).
	dhtt_msgs::msg::Subtree MainServer::construct_subtree_from_node_iter( std::vector<dhtt_msgs::msg::Node>::iterator top_node )
	{
		std::vector<dhtt_msgs::msg::Node> nodes;

		// useful lambda that might be moved elsewhere if I have to write it again
		auto node_name_in_list = [](std::string node_name, std::vector<dhtt_msgs::msg::Node> list)
		{
			for (dhtt_msgs::msg::Node candidate : list)
				if ( not strcmp(candidate.node_name.c_str(), node_name.c_str()) )
					return true; 
			return false;
		};
	
		// lambda to check if a node's parent is in the current list of parents
		auto parent_in_list = [&](dhtt_msgs::msg::Node check) {	return node_name_in_list(check.parent_name, nodes); };

		// deep copy
		auto iter = top_node;

		// find all the parents and construct the subtree
		while (iter != this->node_list.tree_nodes.end())
		{
			nodes.push_back(*iter);
			
			// clear all prior subtree information from node
			nodes.back().parent = -1;
			nodes.back().children = std::vector<int>();

			// now maintain it in the new list
			for (auto parent = nodes.begin(); parent != nodes.end() - 1; parent++)
			{
				if ( not strcmp((*parent).node_name.c_str(), nodes.back().parent_name.c_str()))
				{
					int parent_index = std::distance(nodes.begin(), parent);
					int child_index = std::distance(nodes.begin(), nodes.end() - 1);

					(*parent).children.push_back(child_index);
					
					nodes.back().parent = parent_index;
				}
			}

			// increment otherwise we will just keep adding the same node over and over again
			iter++;

			iter = std::find_if(iter, this->node_list.tree_nodes.end(), parent_in_list);
		}

		// finally construct the message
		dhtt_msgs::msg::Subtree to_ret;
		
		to_ret.tree_nodes = nodes;

		this->fill_subtree_metrics(to_ret);

		return to_ret;
	}

	std::string MainServer::construct_subtree_from_yaml( dhtt_msgs::msg::Subtree& to_construct, std::string file_name, std::vector<std::string> file_args, std::string parent_name )
	{
		std::vector<dhtt_msgs::msg::Node> nodes_to_add;
		std::unordered_map<std::string, std::string> updated_names;

		int number_of_nodes = this->total_nodes_added;

		// yaml-cpp example code taken from here: https://stackoverflow.com/questions/70072926/yaml-cpp-how-to-read-this-yaml-file-content-using-c-linux-using-yaml-cpp-v
		try 
		{

			std::function<std::string ( std::vector<dhtt_msgs::msg::Node>&, std::string, std::vector<std::string>, std::string )> create_nodes_to_add_recursive = 
			[&]( std::vector<dhtt_msgs::msg::Node>& to_construct, std::string file_name_cur, std::vector<std::string> file_args_cur, std::string parent_name_cur) 
			{
				// first build vector of nodes from yaml file
				YAML::Node config = YAML::LoadFile(file_name_cur);
			
				std::vector<std::string> nodes = config["NodeList"].as<std::vector<std::string>>();

				for ( std::vector<std::string>::iterator iter = nodes.begin(); iter != nodes.end(); iter++)
				{
					dhtt_msgs::msg::Node to_build;

					std::string node_name = *iter;
					std::string parent_name = config["Nodes"][(*iter)]["parent"].as<std::string>();

					updated_names[node_name] = node_name + "_" + std::to_string(number_of_nodes);

					to_build.node_name = updated_names[node_name];

					if ( not strcmp(parent_name.c_str(), ROOT_PARENT_NAME) and not strcmp(parent_name_cur.c_str(), "") )
					{
						to_build.parent_name = parent_name;
					}
					else if ( not strcmp(parent_name.c_str(), ROOT_PARENT_NAME) and strcmp(parent_name_cur.c_str(), "") )
					{
						parent_name = parent_name_cur;

						to_build.parent_name = parent_name;
					}
					else
					{
						to_build.parent_name = updated_names[parent_name];
					}

					to_build.type = config["Nodes"][(*iter)]["type"].as<int>();
					to_build.plugin_name = config["Nodes"][(*iter)]["behavior_type"].as<std::string>();

					// check optional parameter "goitr_type"
					try 
					{
						to_build.goitr_name = config["Nodes"][(*iter)]["goitr_type"].as<std::string>();
					}
					catch (const std::exception& e)
					{
						to_build.goitr_name = "";
					}

					// check optional parameter potential_type
					try 
					{
						to_build.potential_type = config["Nodes"][(*iter)]["potential_type"].as<std::string>();
					}
					catch (const std::exception& e)
					{
						to_build.potential_type = "";
					}

					if (to_build.type > dhtt_msgs::msg::Node::SUBTREE)
						return "Node type for node " + to_build.node_name + " incorrect at value " + std::to_string(to_build.type) + ". Returning in error.";

					// also make some consideration for the plugin type but not necessary until that functions

					std::vector<std::string> params = config["Nodes"][(*iter)]["params"].as<std::vector<std::string>>();

					for ( auto param_iter = params.begin(); param_iter != params.end(); param_iter++ )
					{	
						size_t found_index = (*param_iter).find('$');

						// allow for arguments to be passed with the $ character
						if ( found_index != std::string::npos )
						{
							std::string param = (*param_iter).substr(0, found_index - 1);
							std::string arg_to_find = (*param_iter).substr(found_index + 1, ( (*param_iter).length() - found_index ) + 1);

							bool found = false;

							for ( auto arg_iter = file_args_cur.begin(); arg_iter != file_args_cur.end(); arg_iter++ )
							{
								size_t found_index2 = (*arg_iter).find(arg_to_find);

								if ( found_index2 != std::string::npos )
								{
									found = true;

									found_index2 = (*arg_iter).find(':') + 1;

									std::string param_val = (*arg_iter).substr(found_index2, ( (*arg_iter).length() - found_index2 ) + 1);
									// RCLCPP_FATAL(this->get_logger(), "\t\t%s %s", param.c_str(), param_val.c_str());

									to_build.params.push_back(param + param_val);

									break;
								}
							}

							if ( found == false )
								return "Arg from yaml file not found. " + arg_to_find + " should be given a value in the file_args parameter.";
						}
						else
						{
							to_build.params.push_back(*param_iter);
						}
					}

					if ( to_build.type <= dhtt_msgs::msg::Node::BEHAVIOR )
					{
						to_construct.push_back(to_build);

						number_of_nodes++;
					}
					else
					{
						// first extract the file location for the proper path
						std::experimental::filesystem::path file_path = file_name_cur;
						file_path.parent_path();

						to_build.plugin_name = file_path.parent_path().native() + "/" + to_build.plugin_name;

						create_nodes_to_add_recursive(to_construct, to_build.plugin_name, to_build.params, to_build.parent_name);
					}
				}			

				return std::string();
			};

			std::string err = create_nodes_to_add_recursive(nodes_to_add, file_name, file_args, parent_name);

			if ( strcmp(err.c_str(), "") )
				return err;
		}
		catch (const std::exception& exception)
		{
			if ( (int)nodes_to_add.size() > 0 )
				return "Exception [" + std::string(exception.what()) + "] occured after parsing node [" + nodes_to_add.back().node_name.c_str() + "].";
			else
				return "Exception [" + std::string(exception.what()) + "] occured when parsing the first node.";
		}

		// now construct subtree
		std::unordered_map<std::string, int> added_nodes;
		std::unordered_map<std::string, int> depth_by_node;

		int max_width = -1;
		int max_depth = -1;
		int tmp = added_nodes.size();

		int tmp2 = tmp;

		while ( added_nodes.size() < nodes_to_add.size() )
		{

			// first time we can't use the dictionary
			if ( added_nodes.size() == 0 )
			{
				for ( auto iter : nodes_to_add )
				{
					if ( not strcmp( iter.parent_name.c_str(), ROOT_PARENT_NAME ) )
					{
						to_construct.tree_nodes.push_back(iter);
						added_nodes[iter.node_name] = 0;
						depth_by_node[iter.node_name] = 0;

						break;
					}
				}
			}
			else // all other times we are supposed to 
			{
				for ( auto iter : nodes_to_add )
				{	
					if ( added_nodes.find(iter.parent_name) != added_nodes.end() and added_nodes.find(iter.node_name) == added_nodes.end() )
					{
						int parent_index = added_nodes[iter.parent_name];
						depth_by_node[iter.node_name] = depth_by_node[iter.parent_name] + 1;

						if ( depth_by_node[iter.node_name] > max_depth )
							max_depth = depth_by_node[iter.node_name];

						added_nodes[iter.node_name] = to_construct.tree_nodes.size();
						to_construct.tree_nodes.push_back(iter);

						to_construct.tree_nodes[parent_index].children.push_back(added_nodes[iter.node_name]);
						to_construct.tree_nodes[parent_index].child_name.push_back(iter.node_name);

						if ( (int) to_construct.tree_nodes[parent_index].children.size() > max_width )
							max_width = to_construct.tree_nodes[parent_index].children.size();
						
						to_construct.tree_nodes.back().parent = parent_index;
					}
					// else if ( added_nodes.find(iter.parent_name) != added_nodes.end() and added_nodes.find(iter.node_name) != added_nodes.end() )
					// 	return "Parent " + iter.parent_name + " not found for child " + iter.node_name + " returning in error!";
				}
			}

			tmp2 = added_nodes.size();

			if ( tmp == tmp2 )
				return "Node not added during loop. Is the tree connected? Returning in error.";

			tmp = tmp2;
			
		}

		to_construct.max_tree_depth = max_depth;
		to_construct.max_tree_depth = max_depth;
		to_construct.tree_status = dhtt_msgs::msg::NodeStatus::WAITING;

		return "";
	}

	bool MainServer::can_modify(std::string to_modify)
	{
		// in order to modify a node we should make sure it is mutually disjoint from all nodes being modified
		for ( auto being_modified_iter : this->being_modified )
			if ( not this->subtrees_are_disjoint(to_modify, being_modified_iter) )
				return false;

		return true;
	}

	bool MainServer::subtrees_are_disjoint(std::string to_modify, std::string to_check)
	{
		dhtt_msgs::msg::Subtree to_modify_sub = this->fetch_subtree_by_name(to_modify);
		dhtt_msgs::msg::Subtree to_check_sub = this->fetch_subtree_by_name(to_check);

		// if to_modify is in to_check or to_check is in to_modify_sub return false
		for ( auto to_modify_iter : to_modify_sub.tree_nodes )
			if ( not strcmp( to_check.c_str(), to_modify_iter.node_name.c_str() ) )
				return false;

		for ( auto to_check_iter : to_check_sub.tree_nodes )
			if ( not strcmp( to_modify.c_str(), to_check_iter.node_name.c_str() ) )
				return false;

		// otherwise return true
		return true;
	}
}