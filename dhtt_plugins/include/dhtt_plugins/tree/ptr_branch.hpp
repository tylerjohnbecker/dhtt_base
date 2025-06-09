#ifndef PTR_BRANCH_HPP_
#define PTR_BRANCH_HPP_

#include "dhtt/tree/branch_type.hpp"
#include "dhtt/tree/node.hpp"

namespace dhtt_plugins
{
	class PtrBranchSocket : public dhtt::BranchSocketType
	{
	public:
		/**
		 * \brief initializes the pointer to the constructor and nothing else...
		 */
		void initialize(dhtt::Node* constructor_ptr) override;

		/**
		 * \brief just runs the activate function of the attached node
		 */
		dhtt_msgs::action::Activation::Result activation_received_callback(dhtt_msgs::action::Activation::Goal goal) override;

		/**
		 * \brief just runs the maintain conditions function of the attached node
		 */
		dhtt_msgs::action::Condition::Result maintain_received_callback(dhtt_msgs::action::Condition::Goal goal) override;
	};

	class PtrBranchPlug : public dhtt::BranchPlugType
	{
	public:
		/**
		 * \brief saves the ptr to the constructor node and the ptr to the socket
		 * 
		 * This will throw an exception if the socket_ptr is nullptr, otherwise the connection should always be checked with connection_is_good
		 */
		void initialize(dhtt::Node* constructor_ptr, std::shared_ptr<dhtt::BranchSocketType> socket_ptr, std::string child_name) override;

	protected:
		/**
		 * \brief just runs the activation_received_callback in the Socket pair
		 */
		dhtt_msgs::action::Activation::Result send_activate(dhtt_msgs::action::Activation::Goal to_send) override;

		/**
		 * \brief just runs the maintain_received_callback in the Socket pair
		 */
		dhtt_msgs::action::Condition::Result send_maintain(dhtt_msgs::action::Condition::Goal to_send) override;

		/**
		 * \brief checks the connection with the socket and catches exception if mem error occurs with the ptr
		 */
		bool connection_is_good() override;

		std::shared_ptr<dhtt::BranchSocketType> paired_socket;
	};
}

#endif // PTR_BRANCH_HPP_