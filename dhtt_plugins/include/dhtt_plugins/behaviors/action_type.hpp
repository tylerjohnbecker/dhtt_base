#ifndef ACTION_TYPE_HPP
#define ACTION_TYPE_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"

#include <vector>
#include <string>

namespace dhtt_plugins
{

	/**
	 * \brief parent class of all behavior/action type nodes
	 * 
	 * This class is a useful abstraction that removes the necessity for behaviors to all have to generate their own requests for resources. Nodes that inherit from this class just define
	 * 	their necessary resources, what resources they keep/release, and what they do during the work_callback rather than defining everything.
	 */
	class ActionType : public dhtt::NodeType
	{
	public:

		/**
		 * \brief initializer for all action type behaviors
		 * 
		 * Iniatilizes all the relevant flags for a behavior on the tree. Also, initializes a rclcpp::Node and a rclcpp::executors::SingleThreadedExecutor which are meant to be used by the
		 * 	action type behavior to publish/subscribe relevant information and commands to the robot.
		 * 
		 * \param params list of behavior specific params parsed from the yaml description file
		 * 
		 * \return void
		 */
		void initialize(std::vector<std::string> params) override;

		/**
		 * \brief auction callback implementation for all action type behaviors
		 * 
		 * Generates a request for resources based on the necessary_resources member.
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return resulting request for the action server in dhtt::Node
		 */
		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;

		/**
		 * \brief work callback implementation for all action type behaviors
		 * 
		 * Calls the do_work function for action specific work, then generates the result which contains the information about which resources to retain/release based on the defined logic in this
		 * 	class.
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return resulting request for the action server in dhtt::Node
		 */
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container ) override;

		/**
		 * \brief abstract function for actually sending instructions to the robot
		 * 
		 * Every action type behavior performs some work (on the robot or otherwise) which should be implemented in this function. If any topics etc. need to be handled their is an executor
		 * 	that is initialized by default which can be used by inheritors to make those connections. Should flip ActionType.done when work is done.
		 * 
		 * \param container similar to work_callback this is a pointer to the node which is running this logic.
		 * 
		 * \return void
		 */
		virtual void do_work( dhtt::Node* container ) = 0;

		/**
		 * \brief returns a list of all resources that are meant to be retained
		 * 
		 * Retained resources should be kept based on the assumption that the next behavior (only under a THEN node where ordering is guaranteed) will need them. For example, if a robot moves to pick up
		 * 	an object and does not keep control of the base there is a chance that the object would not be picked up which could cause the tree to become uncompletable. Another example is keeping access
		 * 	to an arm after picking up an object so that the arm isn't subsequently used for something else.
		 * 
		 * \param container similar to work_callback this is a pointer to the node which is running this logic.
		 * 
		 * \return list of any retained resources of this behavior
		 */
		virtual std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) = 0;

		/**
		 * \brief returns the list of resources to release
		 * 
		 * This function returns everything other than the resources found in retained.
		 * 
		 * \param container similar to work_callback this is a pointer to the node which is running this logic.
		 * 
		 * \return list of all resources to release 
		 */
		virtual std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node* container );

		/**
		 * \brief this is where inheriting classes define the resources they need
		 * 
		 * The resources on the robot which an action type behavior needs to run should be defined here by overriding this method.
		 * 
		 * \return the list of resources required to run the behavior
		 */
		virtual std::vector<dhtt_msgs::msg::Resource> get_necessary_resources() = 0;

		/**
		 * \brief this is where the inheriting classes define the resources added after execution
		 * 
		 * The resources added should be maintained in the retained and released resources functions, and will be added before those checks in root behavior.
		 * 
		 * \return the list of resources added as a side effect of executing the behavior
		 */
		virtual std::vector<dhtt_msgs::msg::Resource> get_added_resources()
		{
			return add_resources;
		};

		/**
		 * \brief this is where the inheriting classes define the resources destroyed after execution
		 * 
		 * The destroyed resources cannot be retained or released as they will no longer be in the canonical resource list in root behavior by the time those checks occur.
		 * 
		 * \return the list of resources destroyed as a side effect of executing the behavior.
		 */
		virtual std::vector<dhtt_msgs::msg::Resource> get_removed_resources()
		{
			return remove_resources;
		};

		/**
		 * \brief getter for the internal done variable
		 * 
		 * The do_work function should only flip the done flag if work is done, however, if the flag is never flipped this node will be re-run by the tree.
		 * 
		 * \return True if done, false otherwise
		 */
		bool is_done() override;

		/**
		 * \brief create the necessary, added, and removed world resource lists for this behavior
		 * 
		 * This function should be overloaded by all behaviors which require world resources in order to give an interface to define what types are required. The resource lists should be populated with resource types
		 * 	and names will be given at runtime because they are decided by the server.
		 * 
		 * \param container useful for getting the canonical resource list etc.
		 * 
		 * \return void
		 */
		virtual void populate_resource_lists(dhtt::Node* container) { (void) container; };

	protected:

		void send_state_updated();

		bool done;

		double activation_potential;
		std::vector<dhtt_msgs::msg::Resource> necessary_resources; 
		std::vector<dhtt_msgs::msg::Resource> necessary_world_resources;

		std::vector<dhtt_msgs::msg::Resource> add_resources;
		std::vector<dhtt_msgs::msg::Resource> remove_resources;

		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr knowledge_pub;

	private:
	};
}

#endif //ACTION_TYPE_HPP