#include "dhtt/server/goitr_base.hpp"

namespace dhtt
{
	bool GoitrBase::add_node(std::string parent_name, dhtt_msgs::msg::Node to_add)
	{
		(void) parent_name;
		(void) to_add;

		return false;
	}

	bool GoitrBase::remove_node(std::string to_remove)
	{
		(void) to_remove;

		return false;
	}

	bool GoitrBase::change_params(std::string node_name, std::vector<std::string> new_params)
	{
		(void) node_name;
		(void) new_params;

		return false;
	}

	bool GoitrBase::start_servers()
	{
		return false;
	}

	void GoitrBase::modify( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response )
	{
		(void) request;
		(void) response;

		return;
	}

	void GoitrBase::fetch( const std::shared_ptr<dhtt_msgs::srv::FetchRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> response )
	{
		(void) request;
		(void) response;

		return;
	}

	void GoitrBase::control( const std::shared_ptr<dhtt_msgs::srv::ControlRequest::Request> request, const std::shared_ptr<dhtt_msgs::srv::ControlRequest::Response> response)
	{
		(void) request;
		(void) response;

		return;
	}

}