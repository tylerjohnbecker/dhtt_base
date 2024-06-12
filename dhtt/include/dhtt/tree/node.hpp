#ifndef NODE_HPP
#define NODE_HPP

#include <memory>
#include <chrono>

#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "dhtt_msgs/msg/node.hpp"
#include "dhtt_msgs/msg/resource.hpp"
#include "dhtt_msgs/msg/resources.hpp"
#include "dhtt_msgs/msg/node_status.hpp"

#include "dhtt_msgs/srv/internal_service_registration.hpp"
#include "dhtt_msgs/srv/modify_request.hpp"

#include "dhtt_msgs/action/activation.hpp"

#include "dhtt/tree/node_type.hpp"

#define TREE_PREFIX "/dhtt"
#define REGISTER_CHILD_POSTFIX "/register_child"
#define ACTIVATION_POSTFIX "/activate"
#define RESOURCES_POSTFIX "/resource"
#define CONTROL_POSTFIX "/control"

namespace dhtt
{

	/**
	 * \brief dHTT Node class for all nodes on the tree
	 * 
	 * Main Node class for behaviors and tasks on the tree. Facilitates auction behavior, main tree communication protocals, status updates and maintenance, and communication with the Main Server.
	 * 	All node specific logic is kept as a private plugin member that is loaded when the object is created (dhtt::NodeType).
	 */
	class Node : public rclcpp::Node
	{
	public:

		/**
		 * \brief Constructor for the node class
		 * 
		 * Initializes all values simply. If the given plugin name cannot be loaded for some reason does not construct. Errors are flagged through successful_load and error messages are stored in error_msg.
		 * 
		 * 
		 * \param name name of the node being created (should be unique)
		 * \param type exact name of the plugin to load (ex. dhtt_plugins::AndBehavior) 
		 * \param params parameter list for the given plugin (a list of string parameters)
		 * \param parent_name name of this nodes parent (parent is notified of new child through dhtt::MainServer)
		 * 
		 * \return void
		 */
		Node(std::string name, std::string type, std::vector<std::string> params, std::string parent_name);
		~Node();

		friend class MainServer;

		/**
		 * \brief returns the value of successful_load
		 * 
		 * meant for checking whether the plugin was able to be loaded
		 * 
		 * \return True if plugin loaded, false otherwise
		 */
		bool loaded_successfully();

		/**
		 * \brief getter method for error msg
		 * 
		 * \return error message from loading the plugin (blank if no error occurred)
		 */
		std::string get_error_msg();

		/**
		 * \brief Returns all currently owned resources of the node
		 * 
		 * Task and behavior nodes request and then receive resources from the root node of the dHTT. All passed resources are owned as soon as an auction begins.
		 * 
		 * \return vector of owned resources
		 */
		std::vector<dhtt_msgs::msg::Resource> get_owned_resources();

		/**
		 * \brief Returns all resources passed to a node by the parent
		 * 
		 * Resources can be passed to the next node under a THEN node in order to maintain control of them. The message is passed up to the next THEN node and then given to the next
		 * 	child under that node. If the passed resources are sent all the way to the root node then they are released.
		 * 
		 * \return list of passed resources.
		 */
		std::vector<dhtt_msgs::msg::Resource> get_passed_resources();

		/**
		 * \brief Returns the node's copy of the resource state
		 * 
		 * the state of the available resources on the robot is maintained by the root node of the tree. The state is also given to each node in the tree through an internal topic each time
		 * 	the state of the resources is changed. 
		 * 
		 * \return list of all resources on the robot (owned or otherwise)
		 */
		std::vector<dhtt_msgs::msg::Resource> get_resource_state();

		/**
		 * \brief setter method for passsed resources
		 * 
		 * meant to be utilized by plugins in order to pass the resources that should be held. THEN nodes will clear this list, and behavior nodes will set this to owned resources that should be
		 * 	passed. Currently the fidelity of resources is not checked so bugs may occur if these resources don't exist, or are not owned.
		 * 
		 * \param set_to a list of resources to pass
		 * 
		 * \return void
		 */
		void set_passed_resources(std::vector<dhtt_msgs::msg::Resource> set_to);

		/**
		 * \brief registers this node with it's parent
		 * 
		 * sends a service request to the the registration topic of the parent node (found through the given parent name). If this fails then the succesful_load flag is flipped and an error_msg is
		 * 	saved.
		 * 
		 * \return void
		 */
		void register_with_parent();

		/**
		 * \brief registers the necessary servers, publishers, action servers, etc. of this node.
		 * 
		 * should only be used by MainServer. 
		 * 
		 * \return void
		 */
		void register_servers();

		/**
		 * \brief removes a child from this node's list of children
		 * 
		 * the child is already destructed by the MainServer if it is being removed so the only job of this node is to remove it from the list (or forget that it exists).
		 * 
		 * \param child_name name of the child to remove
		 * 
		 * \return True if the child was removed, false otherwise.
		 */
		bool remove_child(std::string child_name);

		// helpers for nodetype
		/**
		 * \brief spreads activation to child without busy wait
		 * 
		 * This is meant to be used to spread activation throughout the tree, then there are specifically designed methods for waiting for responses from multiple children
		 * 
		 * \param child_name name of child to activate
		 * \param activation_goal goal to send to child
		 * 
		 * \return void
		 */
		void async_activate_child(std::string child_name, dhtt_msgs::action::Activation::Goal activation_goal);

		/**
		 * \brief spreads activation to all children
		 * 
		 * iterates through list and utilizes the async_activate_child method
		 * 
		 * \param activation_goal goal to send to children 
		 * 
		 * \return void
		 */
		void activate_all_children(dhtt_msgs::action::Activation::Goal activation_goal);

		/**
		 * \brief busy waits until the number of responses equals the number of activations sent out
		 * 
		 * number of activations is incremented with each call of async_activate_child, number of responses is counted in the store_result_callback
		 * 
		 * \return void
		 */
		bool block_for_responses_from_children();

		/**
		 * \brief getter for the responses from children
		 * 
		 * Should only be called after block_for_responses_from_children. 
		 * 
		 * \return map of child_names to the response they sent back after activating
		 */
		std::map<std::string, dhtt_msgs::action::Activation::Result::SharedPtr> get_activation_results(); 

		/**
		 * \brief getter for the list of child_names
		 * 
		 * \return internal list of children
		 */
		std::vector<std::string> get_child_names();

		/**
		 * \brief getter for the name of the current active child
		 * 
		 * \return either returns the name of the active child or a blank string if no children are active
		 */
		std::string get_active_child_name();

		/**
		 * \brief getter for this node's name as set in the constructor
		 * 
		 * \return the unique name of this node
		 */
		std::string get_node_name();

		/**
		 * \brief checks whether a request of given resources can be fulfilled
		 * 
		 * looks at the internal list of available resources (as given by the root node's message) and the given requested resources. if the list of available resources could be updated
		 * 	to match the request then the request is possible.
		 * 
		 * \param requested_resources list of requested resources (necessary_resources - passed_resources)
		 * 
		 * \return True if the request is possible, false otherwise
		 */
		bool is_request_possible(std::vector<dhtt_msgs::msg::Resource> requested_resources);

		/**
		 * \brief updates the internal status and reports to MainServer
		 * 
		 * just changes the internal state then builds and sends a status message on the /status topic which updates the representation on MainServer. This is the only way that should be
		 * 	used to changed the internal state of a node.
		 * 
		 * \param n_state new state of the node to change to
		 * 
		 * \return void
		 */
		void update_status( int8_t n_state );

		/**
		 * \brief setter for internal resource_status_updated flag
		 * 
		 * This flag is important to check if a request is possible properly (i.e. wait until the available_resources are updated first)
		 * 
		 * \param to_set new value for the resource_status_updated flag
		 * 
		 * \return void
		 */
		void set_resource_status_updated(bool to_set);

	protected:

		// activation action server callbacks
		/**
		 * \brief goal callback for activation action server
		 * 
		 * Just accepts any goal that is given as activation will only be sent by the parent and only one at a time for now. The actual handling of whether the node should become active is 
		 * 	performed in the activation_accepted_callback
		 * 
		 * \param uuid internal action server param (not used in this callback)
		 * \param goal goal passed in for activation (not used in this callback)
		 * 
		 * \return always accept the goal right now
		 */
		rclcpp_action::GoalResponse goal_activation_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const dhtt_msgs::action::Activation::Goal> goal);

		/**
		 * \brief cancel callback for activation action server
		 * 
		 * currently accepts any cancel request. This should perform some logic to immediately stop the node from activating and propogate the cancellation down, however, this is currently 
		 * 	not implemented.
		 * 
		 * \param goal_handle internal action server representation of the goal which is to be cancelled
		 * 
		 * \return always accepts the cancel request for now
		 */
		rclcpp_action::CancelResponse cancel_activation_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle);

		/**
		 * \brief maintains the finite state machine status of the node after activation is received
		 * 
		 * The activation mechanism of a node is described extensively in the paper for dHTT's, however, it essentially does:
		 * - if the node is waiting it becomes active
		 * - if the node is active and gets a successful activation it begins working
		 * - if the node is active and gets a unsuccessful activation it goes back to waiting
		 * - if the node is done it just immediately sends a response and stops
		 * 
		 * if the node becomes active or working it starts a separate thread and runs the activate function. if it goes back to waiting it propogates the unsuccessful activation down to it's 
		 * 	children as well.
		 *  
		 * \param goal_handle internal action server representation of the goal
		 * 
		 * \return void
		 */
		void activation_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle);

		/**
		 * \brief stores results received from activating children
		 * 
		 * Also increments the number of results gotten back so that the busy wait can end after enough responses are received.
		 * 
		 * \param result action server result sent back from child to be saved
		 * \param node_name child from which the result was received (saved when the result callback was bound)
		 * 
		 * \return void
		 */
		void store_result_callback( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Activation>::WrappedResult & result, std::string node_name );

		/**
		 * \brief activates the auction mechanism of task nodes and behavior nodes
		 * 
		 * The major functionality of the activate function is described in the paper on dHTT's extensively. Essentially, this function runs the internal logic of the NodeType plugin for this 
		 * 	node. It performs the following steps:
		 * - if the node is active it runs the auction_callback of the NodeType plugin and then collects and sends back the request 
		 * - if the node is working it runs the work_callback of the NodeType plugin and then releases and passes the appropriate resources
		 * 
		 * \param goal_handle internal action_server representation of the current goal
		 * 
		 * \return void
		 */
		void activate(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle);

		// service callbacks
		/**
		 * \brief registers a new child with this node
		 * 
		 * Adds the name of the child to the list and creates an ActionClient to activate that child (the topic is constructed from the child's name)
		 * 
		 * \param request the InternalServiceRegistration request
		 * \param response the InternalServiceRegistration response
		 * 
		 * \return void
		 */
		void register_child_callback(std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Response> response);

		/**
		 * \brief callback to modify the parameters or nodetype of this node
		 * 
		 * To modify the parameters of a node this method utilizes the NodeType plugin's parse_params function and propogates any error message back in the response. To modify the type the same logic
		 * 	used to create the plugin initially in the constructor is used and the same error_msg is propogated through the response. If the new plugin is not successfully created the old plugin will 
		 * 	be kept instead.
		 * 
		 * \param request ModifyRequest from MainServer
		 * \param response ModifyRequest response. will contain any error_msg from the modification
		 * 
		 * \return void
		 */
		void modify(std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response);

		// subscriber callbacks

		/**
		 * \brief callback for receiving the resource state from root node
		 * 
		 * sets the flag resource_status_updated to true
		 * 
		 * \param canonical_list Resources message sent from the root node.
		 * 
		 * \return void
		 */
		void resource_availability_callback( const dhtt_msgs::msg::Resources::SharedPtr canonical_list );

		// helpful member functions

		/**
		 * \brief checks if the preconditions of the node are met
		 * 
		 * Currently this is not implemented until preconditions are being checked by the tree.
		 * 
		 * \return True always for now
		 */
		bool check_preconditions();

		/**
		 * \brief calculates activation potential as a function of the estimate from the NodeType plugin
		 * 
		 * \deprecated
		 * 
		 * \return activation_potential of the node
		 */
		double calculate_activation_potential();

		/**
		 * \brief sends failure to the currently active child
		 * 
		 * essentially just build a failure activation goal, sends it down to the active child, and blocks for a response.
		 * 
		 * \return void
		 */
		void propogate_failure_down();

		// activation server
		rclcpp_action::Server<dhtt_msgs::action::Activation>::SharedPtr activation_server;

		// activation clients
		std::vector<rclcpp_action::Client<dhtt_msgs::action::Activation>::SharedPtr> activation_clients;
		std::map<std::string, dhtt_msgs::action::Activation::Result::SharedPtr> responses;

		// services
		rclcpp::Service<dhtt_msgs::srv::InternalServiceRegistration>::SharedPtr register_server;

		// members
		pluginlib::ClassLoader<NodeType> node_type_loader;
		std::shared_ptr<NodeType> logic;

		dhtt_msgs::msg::NodeStatus status;
		rclcpp::Publisher<dhtt_msgs::msg::Node>::SharedPtr status_pub;
		rclcpp::Subscription<dhtt_msgs::msg::Resources>::SharedPtr resources_sub;

		std::shared_ptr<std::thread> work_thread;
		std::shared_ptr<std::thread> fail_thread;

		std::mutex logic_mut;

		std::vector<dhtt_msgs::msg::Resource> owned_resources;
		std::vector<dhtt_msgs::msg::Resource> available_resources;
		std::vector<dhtt_msgs::msg::Resource> passed_resources;
		std::vector<std::string> child_names;

		std::string name;
		std::string parent_name;
		std::string plugin_name;

		std::string active_child_name;

		double activation_potential;

		// not sure if i need this
		int activation_level;		
		int stored_responses;
		int expected_responses;

		int priority;

		bool resource_status_updated;

		bool successful_load;
		std::string error_msg;

	private:
	};

}

#endif // NODE_HPP