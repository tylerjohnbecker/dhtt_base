#ifndef MAIN_SERVER_HPP
#define MAIN_SERVER_HPP

// cpp includes
#include <string.h>
#include <iostream>
#include <experimental/filesystem>

// yaml-cpp includes
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/exceptions.h"

// boost includes
#include "boost/thread/mutex.hpp"

// ros2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// dhtt includes
#include "dhtt/tree/node.hpp"

// dhtt message includes
#include "dhtt_msgs/msg/node.hpp"
#include "dhtt_msgs/msg/node_status.hpp"
#include "dhtt_msgs/msg/resource.hpp"
#include "dhtt_msgs/msg/subtree.hpp"

// dhtt srv includes
#include "dhtt_msgs/srv/control_request.hpp"
#include "dhtt_msgs/srv/fetch_request.hpp"
#include "dhtt_msgs/srv/modify_request.hpp"
#include "dhtt_msgs/srv/history_request.hpp"
#include "dhtt_msgs/srv/internal_control_request.hpp"
#include "dhtt_msgs/srv/internal_modify_request.hpp"
#include "dhtt_msgs/srv/internal_service_registration.hpp"

// dhtt action includes
#include "dhtt_msgs/action/activation.hpp"

// helpful constants
#define MAX_NODE_NUM 1000
#define FAILED "failed"
#define ROOT_PARENT -1
#define ROOT_PARENT_NAME "NONE"

namespace dhtt
{
	/** 
	 * \brief main communication to manipulate and get information from the dHTT
	 * 
	 * MainServer contains the dHTT and facilitates any modification requests in four main categories:
	 *   - modification: requests that want to add/remove/etc. from the tree
	 *   - control: requests to start/stop/save the current tree
	 *   - fetch: requests to get specific information from the tree
	 *   - history: requests for runtime history information from the tree
	 */
	class MainServer : public rclcpp::Node
	{
	public:
		/**
		 * \brief constructor for the MainServer Class
		 * 
		 * \param node_name name of the node created by this class
		 * \param spinner outside multithreaded executor which new nodes that are created in the dHTT will be added to
		 */
		MainServer(std::string node_name, std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner);
		~MainServer();

	private:

		// *** PRIVATE MEMBER FUNCTIONS ***
		// server callbacks
		/**
		 * \brief callback for handling ModifyRequests. See ModifyRequest.msg for more information.
		 */ 
		void modify_callback( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response );

		/**
		 * \brief callback for handling ControlRequest. See ControlRequest.msg for more information.
		 */ 
		void control_callback( const std::shared_ptr<dhtt_msgs::srv::ControlRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ControlRequest::Response> response );

		/**
		 * \brief callback for handling FetchRequest. See FetchRequest.msg for more information.
		 */ 
		void fetch_callback( const std::shared_ptr<dhtt_msgs::srv::FetchRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> response );

		/**
		 * \brief callback for handling HistoryRequest. See HistoryRequest.msg for more information.
		 */ 
		void history_callback( const std::shared_ptr<dhtt_msgs::srv::HistoryRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::HistoryRequest::Response> response );

		// modify helpers
		/**
		 * \brief helper function for adding new nodes to the tree
		 * 
		 * Takes a given node and a parent node's name and adds the new node to the parent's list of children. Will throw an exception if the parent doesn't exist,
		 * 	if the parent cannot have a child, or if the child node is not correct. The given plugin for the node type must also exist and be valid.
		 * 
		 * \param response shared_ptr to the response which will be returned to the user. Modified with any successfully added node's name.
		 * \param parent_name name of the parent of the new node
		 * \param to_add msg format of the new node. See Node.msg for more information.
		 * 
		 * \return string with exception information or empty string if successful
		 */
		std::string add_node( std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string parent_name, dhtt_msgs::msg::Node to_add );

		/**
		 * \brief Adds a subtree described in a given .yaml file
		 * 
		 * Takes a yaml file which describes the structure of the desired subtree to the dHTT at the desired parent node (see sample_tasks/pick_place.yaml for an example yaml description).
		 * 	Throws and returns an exception if the file does not exist, has the wrong structure, or if any of the nodes fail to add (see add_node).
		 * 
		 * \param response shared_ptr to the response which will be returned to the user. Modified with any successfully added node's name.
		 * \param parent_name name of the parent of the new node
		 * \param file_name full path of the yaml file which describes the subtree to add
		 * 
		 * \return string with exception information or empty string if successful
		 */
		std::string add_nodes_from_file( std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string parent_name, std::string file_name );

		/**
		 * \brief Removes a given node from the tree
		 * 
		 * Searches the internal list of nodes from the tree, and recursively deletes all children of that node before finally removing the given node. Returns in error if the node does
		 * 	not exist or if the given node is the root node.
		 * 
		 * \param response shared_ptr to the response which will be returned to the user. Modified with any successfully added node's name.
		 * \param to_remove exact name of node to remove from the tree
		 * 
		 * \return string with exception information or empty string if successful
		 */
		std::string remove_node( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string to_remove );

		/**
		 * \brief Reparents \p to_parent to \p new_parent
		 *
		 * Sets the parent field of \p to_parent and calls dhtt::Node::remove_child() on the parent
		 *
		 * \param response shared_ptr to the response which will be returned to the user. Modified with any successfully reparented node's name.
		 * \param to_parent exact name of the node to reparent
		 * \param new_parent exact name of the new parent \p to_parent should become a child of
		 *
		 * \return string with exception information or empty string if successful
		 */
		std::string reparent_node(const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string to_reparent, std::string new_parent);

		/**
		 * \brief Changes params of a given node name
		 * 
		 * NOT IMPLEMENTED YET
		 * 
		 * \return error message
		 */
		std::string change_params( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request );

		// control helpers

		/**
		 * \brief Stops execution of the tree
		 * 
		 * Currently this implementation is in the control callback but should be moved here
		 *   
		 * \param interrupt True if the tree should stop immediately, False for graceful stop. 
		 * 
		 * \return string with exception information or empty string if successful
		 */
		std::string stop_tree( bool interrupt );

		/**
		 * \brief Signals the root node to start execution
		 * 
		 * Starts a new thread of execution which will facilitate the auction behavior described in the root node class. Throws an error if the tree is already running.
		 * 
		 * \return string with exception information or empty string if successful
		 */
		std::string start_tree();

		/**
		 * \brief Saves the current tree and params of each node to a given file name
		 * 
		 * NOT IMPLEMENTED YET
		 * 
		 * \return error message
		 */
		std::string save_tree();

		/**
		 * \brief Deletes everything in the tree except for the root node
		 * 
		 * Removes all children of the root node recursively. Throws an error if the tree is already running.
		 * 
		 * \return  string with exception information or empty string if successful
		 */
		std::string reset_tree();

		// fetch helpers

		/**
		 * \brief fetches all subtrees which contain the given string in their name
		 * 
		 * Returns a list of all Subtrees which match the given name (must contain the literal string in their name). These subtrees can also contain each other so the list may have duplicates. 
		 * 	Subtrees are defined in Subtree.msg see that file for more information.
		 * 
		 * \param name given string to search
		 * 
		 * \return vector of Subtrees which have the given name in common
		 */
		std::vector<dhtt_msgs::msg::Subtree> fetch_subtrees_by_common_name( std::string name );

		/**
		 * \brief fetches all subtrees of a given type
		 * 
		 * Returns a list of all subtrees which are the given type (described in Node.msg). Subtrees can contain each other so there may be duplicates.
		 * 	Subtrees are defined in Subtree.msg see that file for more information.
		 * 
		 * \param type integer value representing the requested type (must be described in Node.msg)
		 * 
		 * \return vector of Subtrees which have the given type
		 */
		std::vector<dhtt_msgs::msg::Subtree> fetch_subtrees_by_type( int type );

		/**
		 * \brief fetches the subtree with exactly the given name
		 * 
		 * Returns only the subtree of the node with the given name, otherwise returns an empty message with status -1 to indicate failure. To ensure the name is exact it should be found from
		 * 	an initial fetch request and then used afterwards as nodes will always be given a unique name upon creation.
		 * 
		 * \param name exact name (case sensitive) of the node to search for in the tree
		 * 
		 * \return subtree with the given name
		 */
		dhtt_msgs::msg::Subtree fetch_subtree_by_name( std::string name );

		// subscriber callbacks
		/**
		 * \brief subscription callback to maintain the internal status of each node
		 * 
		 * Internal subscription which the nodes in the dHTT publish their status to. The status is saved internally in the MainServer and is given in fetch requests as the most up to date version. 
		 * 	Also adds running nodes to the internal history, and specifically publishes the Root Status when a message is received
		 * 
		 * \param data message in the callback
		 * 
		 * \return void
		 */
		void status_callback( const std::shared_ptr<dhtt_msgs::msg::Node> data );

		// generally helpful functions
		/**
		 * \brief Fixes the internal tree representation
		 * 
		 * Makes necessary changes to the internal representation of the dHTT after a node is added or removed.
		 * 
		 * \return void
		 */
		void maintain_local_subtree();

		/**
		 * \brief calculates and fills structural metrics of a given subtree
		 * 
		 * calculates the max width, depth, number of nodes, completion percentage, and current status of the tree and fills in those members of the given message.
		 * 
		 * \param to_fill Subtree message which we have to fill these metrics from
		 * 
		 * \return void
		 */
		void fill_subtree_metrics( dhtt_msgs::msg::Subtree& to_fill );

		/**
		 * \brief subthread function to send the run goal to the dHTT
		 * 
		 * This acquires a mutex to run the tree which is released after the tree finishes execution. 
		 * 
		 * \return void
		 */
		void run_tree();

		/**
		 * \brief callback for when the tree finishes execution
		 * 
		 * Really simple just releases the runtime mutex and flips the running flag
		 * 
		 * \return void
		 */
		void tree_result_callback( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Activation>::WrappedResult & result);

		/**
		 * \brief Function to specifically publish the current status of the root node 
		 * 
		 * This function and the /root_status topic are useful for identifying the current status of the tree as a whole. It just regurgitates the information from the status topic onto the /root_status topic
		 * 	any time the root node publishes a status message.
		 * 
		 * \return void
		 */
		void publish_root_status();

		/**
		 * \brief returns the internal representation of the cucrrent active behavior node
		 * 
		 * This performs a linear search of the nodes in the tree and returns the first that is a BEHAVIOR type node and has an ACTIVE status. Returns a node with name "failed" if the tree if one is not found.
		 * 
		 * \return internal representation of the active node
		 */
		dhtt_msgs::msg::Node get_active_behavior_node();

		/**
		 * \brief Returns a list of all nodes that have status ACTIVE
		 * 
		 * Generally returns the line of nodes from the root node to the active behavior.
		 * 
		 * \return vector of all active nodes in the tree
		 */
		std::vector<dhtt_msgs::msg::Node> get_active_nodes();

		/**
		 * \brief internal function which constructs a subtree message from a given node iter
		 * 
		 * Helpful function which will create a subtree message which represents the subtree of a given node. Generally, this is used for fetch requests.
		 * 
		 * \param top_node iterator of the node to construct the subtree from
		 * 
		 * \return Subtree message representing the given node.
		 */
		dhtt_msgs::msg::Subtree construct_subtree_from_node_iter( std::vector<dhtt_msgs::msg::Node>::iterator top_node );

		// *** PRIVATE MEMBERS ***
		// external services
		rclcpp::Service<dhtt_msgs::srv::ModifyRequest>::SharedPtr modify_server;
		rclcpp::Service<dhtt_msgs::srv::ControlRequest>::SharedPtr control_server;
		rclcpp::Service<dhtt_msgs::srv::FetchRequest>::SharedPtr fetch_server;
		rclcpp::Service<dhtt_msgs::srv::HistoryRequest>::SharedPtr history_server;

		// publishers
		rclcpp::Publisher<dhtt_msgs::msg::NodeStatus>::SharedPtr root_status_pub;

		// internal subscribers
		rclcpp::Subscription<dhtt_msgs::msg::Node>::SharedPtr status_sub;

		rclcpp_action::Client<dhtt_msgs::action::Activation>::SharedPtr client_ptr;
		rclcpp::Client<dhtt_msgs::srv::InternalControlRequest>::SharedPtr internal_control_client;

		// mutexes
		boost::mutex modify_mut;
		boost::mutex running_mut;

		// members
		std::map<std::string, std::shared_ptr<dhtt::Node>> node_map; 
		dhtt_msgs::msg::Subtree node_list; 

		std::shared_ptr<std::thread> run_tree_thread;

		std::list<std::string> history;

		// this is passed in from main through the constructor so that any inner nodes can be spun as well
		std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner_cp;

		int total_nodes_added;
		bool verbose;
		bool running;
	};

}

#endif //MAIN_SERVER_HPP