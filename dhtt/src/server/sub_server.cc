#include "dhtt/server/sub_server.hpp"

namespace dhtt
{
	SubServer::SubServer(std::string node_name, std::string subtree_filename, std::vector<std::string> file_args) : rclcpp::Node(node_name + "_sub_server"), 
			node_name(node_name), service_thread(nullptr), filename(subtree_filename), args(file_args), thread_running(false)
	{

		// this should also request information about it's subtree from main server

		this->start_servers();
	}

	SubServer::~SubServer()
	{
		// if ( this->service_thread != nullptr )
		// 	this->service_thread->join();
	}

	bool SubServer::add_node(std::string parent_name, dhtt_msgs::msg::Node to_add)
	{
		if ( this->thread_running )
			return false; 
		// else if ( this->service_thread != nullptr )
		// 	this->service_thread->join();

		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::ADD;
		req->to_modify.push_back(parent_name);
		req->add_node = to_add;

		this->service_thread = std::make_shared<std::thread>(&SubServer::modify, this, req, res);
		this->service_thread->detach();

		this->thread_running = true;

		return true;
	}

	bool SubServer::remove_node(std::string to_remove)
	{
		if ( this->thread_running )
			return false; 
		// else if ( this->service_thread != nullptr )
		// 	this->service_thread->join();

		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::REMOVE;
		req->to_modify.push_back(to_remove);

		this->service_thread = std::make_shared<std::thread>(&SubServer::modify, this, req, res);
		this->service_thread->detach();

		this->thread_running = true;

		return true;
	}

	bool SubServer::change_params(std::string node_name, std::vector<std::string> new_params)
	{
		if ( this->thread_running )
			return false; 
		// else if ( this->service_thread != nullptr )
		// 	this->service_thread->join();

		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::PARAM_UPDATE;
		req->to_modify.push_back(node_name);
		req->params = new_params;

		this->service_thread = std::make_shared<std::thread>(&SubServer::modify, this, req, res);
		this->service_thread->detach();

		this->thread_running = true;

		return true;
	}

	bool SubServer::start_servers()
	{
		// start each client once
		this->modify_client = this->create_client<dhtt_msgs::srv::ModifyRequest>("/modify_service");

		{
			using namespace std::chrono_literals;

			int time_to_wait = 10;
			int cur_time = 0;

			// wait 10s for the client to connect to the MainServer
			while (!this->modify_client->wait_for_service(1s) and cur_time < time_to_wait)
				cur_time++;

			if (cur_time == time_to_wait)
				return false;
		}
 
		// start topic subscribers

		// start publishers
		this->result_pub = this->create_publisher<dhtt_msgs::msg::Result>(this->node_name + std::string("_sub_server/result"), 10);

		return true;
	}

	bool SubServer::build_subtree()
	{
		if ( this->thread_running )
			return false; 
		// else if ( this->service_thread != nullptr )
		// 	this->service_thread->join();

		// get true path to file saved for subtree
		std::experimental::filesystem::path dhtt_folder_path = __FILE__;

		dhtt_folder_path = dhtt_folder_path.parent_path().parent_path().parent_path();

		std::string true_filename = dhtt_folder_path.native() + "/sample_tasks/" + this->filename;

		// build request for main server
		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::ADD_FROM_FILE;
		req->to_modify.push_back(this->node_name);
		req->to_add = true_filename;
		req->file_args = this->args;

		this->service_thread = std::make_shared<std::thread>(&SubServer::modify, this, req, res);
		this->service_thread->detach();

		this->thread_running = true;
 
		return true;
	}

	bool SubServer::modify(const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response )
	{
		(void) response;

		// rclcpp::sleep_for(std::chrono::milliseconds(10));

		auto future = this->modify_client->async_send_request(request);

		while ( future.wait_for(std::chrono::milliseconds(2)) != std::future_status::ready and rclcpp::ok() );
			
		if (not rclcpp::ok())
			return false;

		response = future.get();

		dhtt_msgs::msg::Result res;
		res.success = response->success;
		res.error_msg = response->error_msg;

		if ( res.success )
			this->update_to_fit_request(request, response);

		this->result_pub->publish(res); 

		this->thread_running = false;

		return response->success;
	}

	bool SubServer::fetch( const std::shared_ptr<dhtt_msgs::srv::FetchRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> response )
	{
		(void) response;

		auto result = this->fetch_client->async_send_request(request);

		if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS)
		{
			// response = result.get();

			return result.get()->success;
		}

		// RCLCPP_ERROR(this->get_logger(), "Could not contact MainServer when changing params of node %s!", request->to_modify[0].c_str());

		return false;
	}

	void SubServer::update_to_fit_request( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response )
	{
		switch( request->type )
		{
		case dhtt_msgs::srv::ModifyRequest::Request::ADD:
			this->child_node_names.push_back(response->added_nodes[0]);
			break;
		case dhtt_msgs::srv::ModifyRequest::Request::REMOVE:
			for ( const auto& removed: response->removed_nodes )
			{
				auto same_name = [&](std::string check) { return not strcmp(check.c_str(), removed.c_str()); };

				std::remove_if(this->child_node_names.begin(), this->child_node_names.end(), same_name);
			}
			break;
		case dhtt_msgs::srv::ModifyRequest::Request::ADD_FROM_FILE:
			for ( auto const& added : response->added_nodes )
				this->child_node_names.push_back(added);
			break;
		}
	}
}