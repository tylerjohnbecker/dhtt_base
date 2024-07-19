#ifndef GOITR_BASE_HPP_
#define GOITR_BASE_HPP_

// cpp includes
#include <string.h>
#include <vector>
#include <memory>

// ros2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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
	protected:

		/**
		 * \brief abstract initializer for GOiTR which will be run after it is created
		 * 
		 * Implementations of this method should ensure that the GOiTR should be fully initialized after this is called. Any extra work (like building the
		 * 	subtree) can be left to the first_activation_callback. Blank for testing in GoitrBase.
		 * 
		 * \param params a vector of params to initialize the node with 
		 * 
		 * \return void
		 */
		virtual void init (std::vector<std::string> params) = 0;

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
		virtual void parent_service(std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Request> req, std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Response> res) = 0;

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
		 * \brief Callback which define how the subtree should be built
		 * 
		 * On first activation by the tree the GOiTR should block and construct it's subtree. This will then happen recursively. The construction should also 
		 * 	take into account the current state of the knowledge base. Blank for testing in GoitrBase.
		 * 
		 * \param TBD
		 * 
		 * \return void
		 */
		virtual void first_activation_callback(/*whatever message for this*/) = 0;

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
		 * \brief helper method to add a node to the tree
		 * 
		 * Communicates with the main server and blocks until a response is given. 
		 * 
		 * \param parent_name name of the node to add the new node to
		 * \param to_add a message description of the node to add to the tree.
		 * 
		 * \return True if adding the node was a success, false otherwise.
		 */
		bool add_node(std::string parent_name, dhtt_msgs::msg::Node to_add);

		/**
		 * \brief helper method to remove a node from the tree
		 * 
		 * Communicates with the main server and blocks until a response is given.
		 * 
		 * \param to_remove name of the node in the tree to remove 
		 * 
		 * \return True if the removal was a success, false otherwise.
		 */
		bool remove_node(std::string to_remove);

		/**
		 * \brief helper method to change parameters of a node in the tree
		 * 
		 * Communicates with the main server and blocks until a response is given. 
		 * 
		 * \param node_name node whose parameters will change
		 * \param new_params new parameters to pass to the node
		 * 
		 * \return true if changing params is successful, false otherwise.
		 */
		bool change_params(std::string node_name, std::vector<std::string> new_params);

		/**
		 * \brief starts the servers for communication with MainServer, KnowledgeBase, and any attached nodes
		 *  
		 * This blocks until communication with all servers has been established.
		 * 
		 * \return True if server communication is established, false otherwise.
		 */
		bool start_servers();

		std::string main_server_topic, node_name; 
		std::vector<std::string> child_node_names;
	private:
		bool modify( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response );
		bool fetch( const std::shared_ptr<dhtt_msgs::srv::FetchRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> response );
		bool control( const std::shared_ptr<dhtt_msgs::srv::ControlRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ControlRequest::Response> response );

		rclcpp::Client<dhtt_msgs::srv::ModifyRequest>::SharedPtr modify_client;
		rclcpp::Client<dhtt_msgs::srv::FetchRequest>::SharedPtr fetch_client;
		rclcpp::Client<dhtt_msgs::srv::ControlRequest>::SharedPtr control_client;

		std::shared_ptr<rclcpp::Node> pub_node_ptr;
		std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
		
	};
}

#endif // GOITR_BASE_HPP_