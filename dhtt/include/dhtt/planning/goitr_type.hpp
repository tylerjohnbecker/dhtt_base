#ifndef GOITR_BASE_HPP_
#define GOITR_BASE_HPP_

// cpp includes
#include <string.h>
#include <vector>
#include <memory>

// ros2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// dhtt includes
#include "dhtt/server/sub_server.hpp"

// dhtt message includes
#include "dhtt_msgs/msg/node.hpp"
#include "dhtt_msgs/msg/pair.hpp"
#include "dhtt_msgs/msg/update_info.hpp"
#include "dhtt_msgs/srv/fetch_info.hpp"
#include "dhtt_msgs/srv/fetch_request.hpp"
#include "dhtt_msgs/srv/modify_request.hpp"
#include "dhtt_msgs/srv/control_request.hpp"
#include "dhtt_msgs/srv/goitr_request.hpp"


namespace dhtt
{
	/**
	 * \brief Goal Oriented in-Task Replanner class
	 * 
	 * The purpose of this class is to provide an interface for building in task replanners that are meant to distribute the task planning problem
	 * 	hierarchically. This interface provides methods for interfacing with the MainServer, Names of nodes that the GOiTR has access to, and abstract
	 * 	callbacks for receiving information from the knowledge base, getting initial activation from the tree, and whenever a child finishes execution.
	 * 	The intent of these callbacks is to provide logic for how replanning should occur.
	 */
	class GoitrType
	{
	public:

		friend class Node;

		/**
		 * \brief base initializer for the GOiTR, runs init_derived as well
		 * 
		 * Implementations of this method should ensure that the GOiTR should be fully initialized after this is called. Any extra work (like building the
		 * 	subtree) can be left to the first_activation_callback. The inheriting class should overload init_derived
		 * 
		 * \param params a vector of params to initialize the node with 
		 * 
		 * \return void
		 */
		void initialize (std::string node_name, std::vector<std::string> params);

		/**
		 * \brief destructor for GOiTR, also runs destruct_derived
		 * 
		 * Implementations of this method should join any threads/clean up any memory problems that could be caused by this node
		 * 
		 * \return void
		 */
		void destruct ();

	protected:
		/**
		 * \brief derived initializer for inheriting classes to initializer their params
		 * 
		 * All GOiTRs will need the initializer function to be run, however, if any additional setup work should be done then it should be put in this function
		 * 	which will also be run in the initialize function after creating the GOiTR
		 * 
		 * \param params a vector of params to initialize the node with 
		 * 
		 * \return void
		 */
		virtual void init_derived(std::string node_name, std::vector<std::string> params);

		/**
		 * \brief derived destructor for inheriting classes to clean up their memory
		 * 
		 * Just cleans up any memory issues in the derived class as well as joins any threads.
		 * 
		 * \return void
		 */
		virtual void destruct_derived();

		/**
		 * \brief service available to parent Goitr to receive information or request changes
		 * 
		 * This service is meant to give a way for the parent to propogate higher level planning choices to these lower level planner. For instance, if a tool is 
		 * 	unavailable or can't be found a subtask which is planning to use it might need to replan so that it is no longer necessary (if possible). This has been
		 * 	given basic functionality in the GoitrBase class for testing.
		 * 
		 * \param req Request from the parent
		 * \param res ptr to give back to the server
		 * 
		 * \return void
		 */
		virtual void parent_service_callback(std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Request> req, std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Response> res) = 0;

		/**
		 * \brief Callback which defines the logic for when knowledge updates are received
		 * 
		 * The purpose of this callback is to enable the GOiTR to continuously replan whenever knew information is received from the knowledge base. New knowledge
		 * 	that requires could include the changing location of an object, the availability of a tool, etc. Blank for testing in GoitrBase.
		 * 
		 * \param TBD
		 * 
		 * \return void
		 */
		virtual void knowledge_update_callback(/*whatever message for this*/) = 0;

		/**
		 * \brief Callback which defines how to act when a child that this node has direct access to has finished.0
		 * 
		 * When children finish successfully this callback may have changes to the tree. Specifically, if a child node fails this callback should assess the reason
		 * 	that the node failed and replan the subtree accordingly. Blank for testing in GoitrBase.
		 * 
		 * \param TBD
		 * 
		 * \return void
		 */
		virtual void child_finished_callback(/*whatever message for this*/) = 0;

		/**
		 * \brief Callback which define how the subtree should be built
		 * 
		 * On first activation by the tree the GOiTR should block and construct it's subtree. This will then happen recursively. The construction should also 
		 * 	take into account the current state of the knowledge base. Blank for testing in GoitrBase.
		 * 
		 * \param TBD
		 * 
		 * \return void
		 */
		virtual void first_activation_callback(/*whatever message for this*/);

		/**
		 * \brief starts services, publications, and subscriptions for this node
		 * 
		 * All subscriptions are made through the sub_server node and are handled in a SingleThreadedExecutor for now. If more threads are required in the future that
		 * 	may change, however, for now it seems that there would be some mistake in the implementation if that becomes an issue.
		 * 
		 * \return bool True if servers start correctly, false otherwise
		 */
		bool start_servers();

		std::shared_ptr<SubServer> sub_srv_ptr;
		std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;

		rclcpp::Service<dhtt_msgs::srv::GoitrRequest>::SharedPtr parent_service;

		std::string main_server_topic, node_name; 
		std::vector<std::string> child_node_names;

		bool tree_built;

	private:

		void async_spin();

		std::shared_ptr<std::thread> spin_thread;
		bool keep_spinning;

	};
}

#endif // GOITR_BASE_HPP_