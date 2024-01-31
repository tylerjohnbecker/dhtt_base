#include "dhtt/server/main_server.hpp"

namespace dHTT
{
	MainServer::MainServer(std::string node_name) : rclcpp::Node(node_name), total_nodes_added(1), verbose(true)
	{
		/// initialize subscribers
		this->status_sub = this->create_subscription<dhtt_msgs::msg::Node>("/status", MAX_NODE_NUM, std::bind(&MainServer::status_callback, this, std::placeholders::_1));

		/// initialize publishers
		this->root_status_pub = this->create_publisher<dhtt_msgs::msg::NodeStatus>("/root_status", 10);

		/// initialize internal services
		this->registration_server = this->create_service<dhtt_msgs::srv::InternalServiceRegistration>("/register_service", std::bind(&MainServer::register_callback, this, std::placeholders::_1, std::placeholders::_2));

		/// make root node (before external services because they will not work without an active root)
		// make local representation
		dhtt_msgs::msg::Node::SharedPtr root_node = std::make_shared<dhtt_msgs::msg::Node>();

		root_node->node_name = "ROOT_0";
		root_node->parent = ROOT_PARENT;
		root_node->parent_name = std::string("NONE");

		root_node->children = std::vector<int>();
		root_node->child_name = std::vector<std::string>();

		root_node->params = std::vector<std::string>();

		root_node->type = dhtt_msgs::msg::Node::ROOT;

		root_node->node_status.state = dhtt_msgs::msg::NodeStatus::WAITING;

		this->node_list.tree_nodes.push_back(*root_node);
		this->node_list.tree_status = dhtt_msgs::msg::NodeStatus::WAITING;
		this->node_list.max_tree_depth = 0;
		this->node_list.max_tree_width = 1;

		this->node_list.task_completion_percent = 0.0f;

		// initialize physical Root Node
		// need class representation first

		/// initialize external services
		this->modify_server = this->create_service<dhtt_msgs::srv::ModifyRequest>("/modify_service", std::bind(&MainServer::modify_callback, this, std::placeholders::_1, std::placeholders::_2));
		this->control_server = this->create_service<dhtt_msgs::srv::ControlRequest>("/control_service", std::bind(&MainServer::control_callback, this, std::placeholders::_1, std::placeholders::_2));
		this->fetch_server = this->create_service<dhtt_msgs::srv::FetchRequest>("/fetch_service", std::bind(&MainServer::fetch_callback, this, std::placeholders::_1, std::placeholders::_2));
	}

	MainServer::~MainServer()
	{
		// just manually cleaning up the data structure here
		this->verbose = false;

		// not functioning at the moment, will have to revisit this when I get to the proper node structure
		// this->remove_node(std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>(), this->node_list.tree_nodes[0]);
	}

	void MainServer::modify_callback( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response )
	{
		RCLCPP_INFO(this->get_logger(), "--- Modify request received!");

		if ( request->type == dhtt_msgs::srv::ModifyRequest::Request::ADD)
		{
			// docs aren't great, but search scoped locks here: https://www.boost.org/doc/libs/1_65_0/doc/html/thread/synchronization.html
			std::lock_guard<boost::mutex> guard(this->modify_mut);

			if ( (int) request->to_modify.size() == 0 )
			{
				response->success = false;
				response->error_msg = "Failed to add node because no parent node was given."; 

				RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

				return;
			}

			for (auto iter = request->to_modify.begin(); iter != request->to_modify.end(); iter++)
			{
				response->error_msg = this->add_node(response, *iter, request->add_node);
				response->success = not strcmp(response->error_msg.c_str(), "");

				// exit early if modification fails
				if ( not response->success )
				{
					RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

					return;
				}
			}

			return;
		}

		if ( request->type == dhtt_msgs::srv::ModifyRequest::Request::ADD_FROM_FILE)
		{
			std::lock_guard<boost::mutex> guard(this->modify_mut);
			
			if ( (int) request->to_modify.size() == 0 )
			{
				response->success = false;
				response->error_msg = "Failed to add nodes from file because no parent node was given."; 

				RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

				return;
			}

			for (auto iter = request->to_modify.begin(); iter != request->to_modify.end(); iter++)
			{
				// currently this opens the yaml file n times where n is the number of placesthat we add the nodes
				// in practice we should have negligible time loss from this but it should technically be done first before the loop
				// and then the dictionary should be passed in
				response->error_msg = this->add_nodes_from_file(response, *iter, request->to_add);
				response->success = not strcmp(response->error_msg.c_str(), "");

				// exit early if modification fails
				if ( not response->success )
				{
					RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

					return;
				}
			}

			return;
		}

		if ( request->type == dhtt_msgs::srv::ModifyRequest::Request::REMOVE )
		{
			std::lock_guard<boost::mutex> guard(this->modify_mut);

			if ( (int) request->to_modify.size() == 0 )
			{
				response->success = false;
				response->error_msg = "Failed to remove nodes because no nodes to remove were given.";
				
				RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

				return;
			}

			for (auto iter = request->to_modify.begin(); iter != request->to_modify.end(); iter++)
			{
				response->error_msg = this->remove_node(response, *iter);
				response->success = not strcmp(response->error_msg.c_str(), "");

				// exit early if modification fails
				if ( not response->success )
				{
					RCLCPP_ERROR(this->get_logger(), "%s", response->error_msg.c_str());

					return;
				}
			}

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


		// reset
		if ( request->type == dhtt_msgs::srv::ControlRequest::Request::RESET )
		{
			response->error_msg = this->reset_tree();
			response->success = not strcmp(response->error_msg.c_str(), "");

			return;
		}
	}
	
	void MainServer::fetch_callback( const std::shared_ptr<dhtt_msgs::srv::FetchRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> response )
	{

		RCLCPP_INFO(this->get_logger(), "--- Fetch request recieved!");

		if ( request->return_full_subtree )
		{
			response->found_subtrees.push_back(this->construct_subtree_from_node_iter(this->node_list.tree_nodes.begin()));
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
	
	void MainServer::register_callback( const std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Response> response )
	{
		(void) request;
		(void) response;

	}

	// modify helpers
	// I should also think about the node number here
	std::string MainServer::add_node( std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string parent_name, dhtt_msgs::msg::Node to_add )
	{
		auto is_parent = [&]( dhtt_msgs::msg::Node check ) { return not strcmp(parent_name.c_str(), check.node_name.c_str()); };

		std::vector<dhtt_msgs::msg::Node>::iterator found_parent = std::find_if( this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), is_parent);

		if ( ( to_add.type < dhtt_msgs::msg::Node::AND ) or ( to_add.type > dhtt_msgs::msg::Node::BEHAVIOR ) )
		{
			return "Can\'t add a behavior with type " + std::to_string(to_add.type) + ". Returning in error.";
		}

		if ( found_parent == this->node_list.tree_nodes.end() )
		{
			return "Parent " + parent_name + " not found in tree. Returning in error.";
		}

		if ( found_parent->type >= dhtt_msgs::msg::Node::BEHAVIOR )
		{
			return "Parent cannot be a BEHAVIOR type node. Returning in error!";
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
		
		// create a physical node from the message and add to physical list

		// update the parent
		(*found_parent).children.push_back( (int) this->node_list.tree_nodes.size() );
		(*found_parent).child_name.push_back(to_add.node_name);

		this->node_list.tree_nodes.push_back(to_add);

		// notify physical parent to update it's children with an internalmodifyrequest


		// add this node_name to the list of added nodes
		response->added_nodes.push_back(to_add.node_name);

		// finally add to the counter so that we continue to get unique names (up until 2^64 nodes are added of course)
		this->total_nodes_added++;

		return "";
	}

	std::string MainServer::add_nodes_from_file(std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string parent_name, std::string file_name )
	{
		// first load all of the yaml nodes into the message format
		std::vector<dhtt_msgs::msg::Node> nodes_to_add;

		// yaml-cpp example code taken from here: https://stackoverflow.com/questions/70072926/yaml-cpp-how-to-read-this-yaml-file-content-using-c-linux-using-yaml-cpp-v
		try 
		{
			YAML::Node config = YAML::LoadFile(file_name);
		
			std::vector<std::string> nodes = config["NodeList"].as<std::vector<std::string>>();

			for ( std::vector<std::string>::iterator iter = nodes.begin(); iter != nodes.end(); iter++)
			{
				dhtt_msgs::msg::Node to_build;

				to_build.node_name = *iter;
				to_build.parent_name = config["Nodes"][(*iter)]["parent"].as<std::string>();

				to_build.type = config["Nodes"][(*iter)]["type"].as<int>();

				if (to_build.type > dhtt_msgs::msg::Node::BEHAVIOR)
					return "Node type for node " + to_build.node_name + " incorrect at value " + std::to_string(to_build.type) + ". Returning in error.";

				// also make some consideration for the plugin type but not necessary until that functions

				std::vector<std::string> params = config["Nodes"][(*iter)]["params"].as<std::vector<std::string>>();

				for ( auto param_iter = params.begin(); param_iter != params.end(); param_iter++ )
				{
					to_build.params.push_back(*param_iter);
				}

				nodes_to_add.push_back(to_build);
			}			
		}
		catch (const std::exception& exception)
		{
			return exception.what();
		}

		if (this->verbose)
			RCLCPP_INFO(this->get_logger(), "Adding subtree loaded from file %s", file_name.c_str());

		// then use the add_node function which is already meant to work with node msgs
		for ( std::vector<dhtt_msgs::msg::Node>::iterator iter = nodes_to_add.begin(); iter != nodes_to_add.end(); iter++ )
		{
			std::string old_name = (*iter).node_name;
			std::string p_name = (*iter).parent_name;

			if ( not strcmp(p_name.c_str(), ROOT_PARENT_NAME ) )
			{
				p_name = parent_name;

				(*iter).parent_name = parent_name;
			}

			std::string error_msg = this->add_node(response, p_name, (*iter));

			// make sure to pass the error through if anything fails
			if ( strcmp(error_msg.c_str(), "") )
				return error_msg;

			// because we change the name to a unique id when it is added we also need to update the parent names of the nodes we are adding
			std::string new_name = this->node_list.tree_nodes.back().node_name;

			for ( std::vector<dhtt_msgs::msg::Node>::iterator name_update_iter = iter + 1; name_update_iter != nodes_to_add.end(); name_update_iter++ )
				if ( not strcmp( (*name_update_iter).parent_name.c_str(), old_name.c_str() ) )
					(*name_update_iter).parent_name = new_name;
		}

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

			response->removed_nodes.push_back(look_for);
		}

		if ( (int) this->node_list.tree_nodes.size() == 0 )
			return "";

		// instead they just have to be maintained after the subtree is removed
		this->maintain_local_subtree();

		return "";
	}
	
	std::string MainServer::change_params( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request )
	{
		(void) request;

		return "";
	}

	// control helpers
	std::string MainServer::stop_tree( bool interrupt )
	{
		(void) interrupt;

		return "";
	}

	std::string MainServer::start_tree()
	{
		return "";
	}

	std::string MainServer::save_tree()
	{
		return "";
	}

	std::string MainServer::reset_tree()
	{

		if ( this->node_list.tree_status == dhtt_msgs::msg::NodeStatus::ACTIVE or this->node_list.tree_status == dhtt_msgs::msg::NodeStatus::WORKING )
		{
			return "Can\'t reset the tree while it is active. Returning in error.";
		}

		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> blank_rs = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		for ( std::vector<std::string>::iterator iter = this->node_list.tree_nodes[0].child_name.begin() ; iter != this->node_list.tree_nodes[0].child_name.end() ; iter++ )
		{
			this->remove_node( blank_rs, *iter );
		}

		return "";
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

	// subscriber callbacks
	void MainServer::status_callback( const std::shared_ptr<dhtt_msgs::msg::Node> data )
	{
		// unary lambda function for find_if which takes advantage of the scope inside this function. This function is not useful elsewhere
		auto same_name = [&](dhtt_msgs::msg::Node check) { return not strcmp(data->node_name.c_str(), check.node_name.c_str()); };

		auto found_node = std::find_if(this->node_list.tree_nodes.begin(), this->node_list.tree_nodes.end(), same_name);

		// node with matching name was not found
		if ( found_node == this->node_list.tree_nodes.end() )
			return;

		(*found_node).node_status.state = data->node_status.state;
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

			(*parent_iter).children[child_index] = std::distance(this->node_list.tree_nodes.rbegin(), node_iter) - 1;
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

		for (dhtt_msgs::msg::Node n : this->node_list.tree_nodes)

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
}