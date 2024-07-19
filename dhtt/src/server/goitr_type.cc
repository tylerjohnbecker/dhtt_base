#include "dhtt/server/goitr_type.hpp"

namespace dhtt
{

	bool GoitrType::add_node(std::string parent_name, dhtt_msgs::msg::Node to_add)
	{
		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::ADD;
		req->to_modify.push_back(parent_name);
		req->add_node = to_add;

		return this->modify(req, res);
	}

	bool GoitrType::remove_node(std::string to_remove)
	{

		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::REMOVE;
		req->to_modify.push_back(to_remove);

		return this->modify(req, res);
	}

	bool GoitrType::change_params(std::string node_name, std::vector<std::string> new_params)
	{
		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> req = std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
		std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

		req->type = dhtt_msgs::srv::ModifyRequest::Request::PARAM_UPDATE;
		req->to_modify.push_back(node_name);
		req->params = new_params;

		return this->modify(req, res);
	}

	bool GoitrType::start_servers()
	{

		// just a random hopefully unique name
		const void * address = static_cast<const void*>(this);
		std::stringstream ss;
		ss << "goitr_" << address;

		// make a node and executor pair
		this->pub_node_ptr = std::make_shared<rclcpp::Node>(ss.str().c_str());
		this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
		this->executor->add_node(this->pub_node_ptr);

		// start each client once
		this->modify_client = this->pub_node_ptr->create_client<dhtt_msgs::srv::ModifyRequest>("/modify_service");

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

		return true;
	}

	bool GoitrType::modify( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response )
	{
		(void) response;

		auto result = this->modify_client->async_send_request(request);

		if (rclcpp::spin_until_future_complete(this->pub_node_ptr->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS)
		{
			// response = result.get();

			return result.get()->success;
		}

		RCLCPP_ERROR(this->pub_node_ptr->get_logger(), "Could not contact MainServer when modifying node %s!", request->to_modify[0].c_str());

		return false;
	}

	bool GoitrType::fetch( const std::shared_ptr<dhtt_msgs::srv::FetchRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> response )
	{
		(void) response;

		auto result = this->fetch_client->async_send_request(request);

		if (rclcpp::spin_until_future_complete(this->pub_node_ptr->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS)
		{
			// response = result.get();

			return result.get()->success;
		}

		// RCLCPP_ERROR(this->get_logger(), "Could not contact MainServer when changing params of node %s!", request->to_modify[0].c_str());

		return false;
	}

	bool GoitrType::control( const std::shared_ptr<dhtt_msgs::srv::ControlRequest::Request> request, const std::shared_ptr<dhtt_msgs::srv::ControlRequest::Response> response)
	{
		(void) response;

		auto result = this->control_client->async_send_request(request);

		if (rclcpp::spin_until_future_complete(this->pub_node_ptr->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS)
		{
			// response = result.get();

			return result.get()->success;
		}

		// RCLCPP_ERROR(this->get_logger(), "Could not contact MainServer when changing params of node %s!", request->to_modify[0].c_str());

		return false;
	}

}