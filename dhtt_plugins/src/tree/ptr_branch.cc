#include "dhtt_plugins/tree/ptr_branch.hpp"

namespace dhtt_plugins
{
	void PtrBranchSocket::initialize(dhtt::Node* constructor_ptr)
	{
		this->container = constructor_ptr;
	}

	dhtt_msgs::action::Activation::Result PtrBranchSocket::activation_received_callback(dhtt_msgs::action::Activation::Goal goal)
	{
		return this->container->activate(goal);
	}

	dhtt_msgs::action::Condition::Result PtrBranchSocket::maintain_received_callback(dhtt_msgs::action::Condition::Goal goal)
	{
		return this->container->combine_child_conditions(goal);
	}

	void PtrBranchPlug::initialize(dhtt::Node* constructor_ptr, std::shared_ptr<dhtt::BranchSocketType> socket_ptr, std::string child_name)
	{
		this->container = constructor_ptr;
		this->paired_socket = socket_ptr;

		(void) child_name;
	}

	dhtt_msgs::action::Activation::Result PtrBranchPlug::send_activate(dhtt_msgs::action::Activation::Goal to_send)
	{
		return this->paired_socket->activation_received_callback(to_send);
	}

	dhtt_msgs::action::Condition::Result PtrBranchPlug::send_maintain(dhtt_msgs::action::Condition::Goal to_send)
	{
		return this->paired_socket->maintain_received_callback(to_send);
	}

	bool PtrBranchPlug::connection_is_good()
	{
		if ( this->paired_socket == nullptr )
			return false;

		return this->paired_socket->test_connection();
	}
}