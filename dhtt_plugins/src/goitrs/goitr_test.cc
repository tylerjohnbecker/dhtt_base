#include "dhtt_plugins/goitrs/goitr_test.hpp"

namespace dhtt_plugins
{

	void GoitrTest::parent_service_callback(std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Request> req, std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Response> res)
	{
		std::string to_find;

		RCLCPP_INFO(this->sub_srv_ptr->get_logger(), "--- Goitr Request recieved!");

		auto find_by_name = [&](std::string to_check)
		{
			return not strcmp(to_check.c_str(), to_find.c_str());
		};

		if ( req->type == dhtt_msgs::srv::GoitrRequest::Request::ADD ) 
		{
			// we need at least 2 params to create the node (parent_name, node_name, type, goitr_type). The rest in the params list will become the params for the new node
			if ( (int) req->params.size() < 4)
			{
				res->success = false;
				res->error_msg = "Not enough parameters given to create node. Returning in error!";

				return;
			}

			auto found = this->child_node_names.begin();

			to_find = req->params[0];

			if ( not strcmp( req->params[0].c_str(), "NONE" ) )
			{
				to_find = this->node_name;
			}
			else if ( ( found = std::find_if(this->sub_srv_ptr->child_node_names.begin(), this->sub_srv_ptr->child_node_names.end(), find_by_name ) ) == this->sub_srv_ptr->child_node_names.end() )
			{
				// for ( const auto& iter : this->sub_srv_ptr->child_node_names )
				// 	RCLCPP_ERROR(this->sub_srv_ptr->get_logger(), "I have abandoned my child! %s", iter.c_str());

				res->success = false;
				res->error_msg = "Node " + req->params[0] + " not found! Returning in error.";

				return;
			}

			dhtt_msgs::msg::Node to_add;

			to_add.parent_name = to_find;
			to_add.node_name = req->params[1];
			to_add.plugin_name = req->params[2];
			to_add.goitr_name = req->params[3];
			to_add.type = dhtt_msgs::msg::Node::BEHAVIOR; // only so the server is happy during testing

			for ( auto iter = req->params.begin() + 4; iter < req->params.end(); iter++ )
				to_add.params.push_back(*iter);

			// not sure I can do this in the callback (we'll find out)
			res->success = this->sub_srv_ptr->add_node(to_find, to_add);

			if ( not res->success )
				res->error_msg = "Failed to add node";

			// a bit cleaner to return here
			return;
		}

		else if ( req->type == dhtt_msgs::srv::GoitrRequest::Request::REMOVE )
		{
			if ( (int) req->params.size() < 1 )
			{
				res->success = false;
				res->error_msg = "Not enough parameters given to remove node! Returning in error.";
			}

			auto found = this->child_node_names.begin();

			to_find = req->params[0];

			if ( ( found = std::find_if(this->sub_srv_ptr->child_node_names.begin(), this->sub_srv_ptr->child_node_names.end(), find_by_name ) ) == this->sub_srv_ptr->child_node_names.end() )
			{
				// for ( const auto& iter : this->sub_srv_ptr->child_node_names )
				// 	RCLCPP_ERROR(this->sub_srv_ptr->get_logger(), "I have abandoned my child! %s", iter.c_str());

				res->success = false;
				res->error_msg = "Node " + req->params[0] + " not found! Returning in error.";

				return;
			}

			// not sure I can do this in the callback (we'll find out)
			res->success = this->sub_srv_ptr->remove_node(req->params[0]);

			if ( not res->success )
				res->error_msg = "Failed to remove node";
		}

		else if ( req->type == dhtt_msgs::srv::GoitrRequest::Request::CHANGE_PARAMS )
		{
			if ( (int) req->params.size() < 2 )
			{
				res->success = false;
				res->error_msg = "Not enough parameters given to remove node! Returning in error.";
			}

			auto found = this->child_node_names.begin();

			to_find = req->params[0];

			if ( ( found = std::find_if(this->sub_srv_ptr->child_node_names.begin(), this->sub_srv_ptr->child_node_names.end(), find_by_name ) ) == this->sub_srv_ptr->child_node_names.end() )
			{
				res->success = false;
				res->error_msg = "Node " + req->params[0] + " not found! Returning in error.";

				return;
			}

			std::vector<std::string> n_params;

			for ( auto iter = req->params.begin() + 1; iter < req->params.end(); iter++ )
				n_params.push_back(*iter); 

			// not sure I can do this in the callback (we'll find out)
			res->success = this->sub_srv_ptr->change_params(req->params[0], n_params);

			if ( not res->success )
				res->error_msg = "Failed to change params of node";
		}

		else if ( req->type == dhtt_msgs::srv::GoitrRequest::Request::BUILD_TREE )
		{
			if ( this->tree_built )
			{
				res->success = false;
				res->error_msg = "Failed to build tree because tree has already been built!";

				return;
			}

			res->success = this->sub_srv_ptr->build_subtree();
			this->tree_built = res->success;

			if ( not res->success )
				res->error_msg = "Tree failed to build. Returning in error...";
		}

		else
		{
			res->success = false;
			res->error_msg = "Invalid message type: " + std::to_string(req->type);
		}
	}

	void GoitrTest::knowledge_update_callback(std::shared_ptr<std_msgs::msg::String> updated_knowledge)
	{
		// wow such empty
	}

	void GoitrTest::first_activation_callback(/*whatever message for this*/)
	{
		// wow such empty
	}

	void GoitrTest::child_finished_callback(std::string finished_child, bool success)
	{
		(void) finished_child;
	}
}