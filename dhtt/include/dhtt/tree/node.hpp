#ifndef NODE_HPP
#define NODE_HPP

#include <memory>
#include <chrono>
#include <condition_variable>
#include <mutex>

#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"

#include "dhtt_msgs/msg/node.hpp"
#include "dhtt_msgs/msg/resource.hpp"
#include "dhtt_msgs/msg/resources.hpp"
#include "dhtt_msgs/msg/node_status.hpp"

#include "dhtt_msgs/srv/internal_service_registration.hpp"
#include "dhtt_msgs/srv/modify_request.hpp"

#include "dhtt_msgs/action/activation.hpp"
#include "dhtt_msgs/action/condition.hpp"

// interfaces
#include "dhtt/tree/node_type.hpp"
#include "dhtt/tree/branch_type.hpp"
#include "dhtt/tree/potential_type.hpp"
#include "dhtt/planning/goitr_type.hpp"

// dhtt includes
#include "dhtt/server/communication_aggregator.hpp"

#define TREE_PREFIX "/dhtt"
#define REGISTER_CHILD_POSTFIX "/register_child"
#define ACTIVATION_POSTFIX "/activate"
#define CONDITION_POSTFIX "/condition"
#define RESOURCES_POSTFIX "/resource"
#define CONTROL_POSTFIX "/control"

#define ACTIVATION_POTENTIAL_HIGHEST 2.0


// #define DHTT_LOG_INFO(com_ptr, string_stream) com_ptr->log_stream(1, std::stringstream() << "[" << this->name << "]: " << string_stream)
// #define DHTT_LOG_WARN(com_ptr, string_stream) com_ptr->log_stream(2, std::stringstream() << "[" << this->name << "]: " << string_stream)
// #define DHTT_LOG_ERROR(com_ptr, string_stream) com_ptr->log_stream(3, std::stringstream() << "[" << this->name << "]: " << string_stream)
// #define DHTT_LOG_FATAL(com_ptr, string_stream) com_ptr->log_stream(4, std::stringstream() << "[" << this->name << "]: " << string_stream)

namespace dhtt
{

#define DHTT_LOG(severity, com_ptr, out) \
	{ \
		std::stringstream ss; \
		ss <<  "[" << this->name << "]: " << out; \
		com_ptr->log_stream(severity,  ss); \
	}

#define DHTT_LOG_DEBUG(com_ptr, out) DHTT_LOG(dhtt::LOG_LEVEL::DEBUG, com_ptr, out)
#define DHTT_LOG_INFO(com_ptr, out) DHTT_LOG(dhtt::LOG_LEVEL::INFO, com_ptr, out)
#define DHTT_LOG_WARN(com_ptr, out) DHTT_LOG(dhtt::LOG_LEVEL::WARN, com_ptr, out)
#define DHTT_LOG_ERROR(com_ptr, out) DHTT_LOG(dhtt::LOG_LEVEL::ERROR, com_ptr, out)
#define DHTT_LOG_FATAL(com_ptr, out) DHTT_LOG(dhtt::LOG_LEVEL::FATAL, com_ptr, out)

	/**
	 * \brief dHTT Node class for all nodes on the tree
	 * 
	 * Main Node class for behaviors and tasks on the tree. Facilitates auction behavior, activation spreading behavior, basic resource management, status updates and maintenance, and communication with the Main Server.
	 * 	All node specific logic is kept as a private plugin member that is loaded when the object is created (dhtt::NodeType). All node communication mechanisms are also kept as a private plugin member so that it can
	 * 	be modularized.
	 */
	class Node
	{
	public:

		/**
		 * \brief Constructor for the node class
		 * 
		 * Initializes all values simply. If the given plugin names cannot be loaded for some reason does not construct. Errors are flagged through successful_load and error messages are stored in error_msg.
		 * 
		 * \param name name of the node being created (should be unique)
		 * \param type exact name of the plugin to load (ex. dhtt_plugins::AndBehavior) 
		 * \param params parameter list for the given plugin (a list of string parameters)
		 * \param parent_name name of this nodes parent (parent is notified of new child through dhtt::MainServer)
		 * \param socket_type name of the plugin to use for this node's parent socket
		 * \param goitr_type name of the goitr plugin to load for this node (if empty no goitr is loaded)
		 * 
		 * \return void
		 */
		Node(std::shared_ptr<CommunicationAggregator> com_agg, std::string name, std::string type, std::vector<std::string> params, std::string parent_name, std::string socket_type="dhtt_plugins::PtrBranchSocket", std::string goitr_type="", std::string potential_type="dhtt_plugins::EfficiencyPotential");
		~Node();

		// Node(Node const&)=delete;// deleting copy constructor to debug

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
		 * \brief get the number of resources owned in the subtree starting at this node
		 * 
		 * \return number of resource used by this subtree
		 */
		int get_subtree_resources();

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
		 * \brief returns a shared ptr to the socket_ptr of this node
		 * 
		 * Mostly used for constructing the corresponding plug but could be useful outside of this case.
		 * 
		 * \return shared_ptr to this node's parent socket
		 */
		std::shared_ptr<BranchSocketType> get_socket_ptr();

		/**
		 * \brief returns a shared ptr to the global communication aggregator for the tree
		 * 
		 * This is meant so that the logic and other plugins can create publishers and subscribers in a way that is scalable with respect to the DDS ROS uses. Otherwise
		 * 	there is a hard upper limit on the amount of subscriptions allowed per node for example.
		 * 
		 * \return shared ptr to the global communication aggregator
		 */
		std::shared_ptr<CommunicationAggregator> get_com_agg();

		/**
		 * \brief returns a ptr to the logic plugin of this node
		 * 
		 * This is useful for making considerations for activation potential for example from outside of this node.
		 * 
		 * \return a ptr to the logic plugin of this node
		 */
		std::shared_ptr<NodeType> get_logic();

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
		// void register_with_parent(int index=-1);

		/**
		 * \brief registers the necessary servers, publishers, action servers, etc. of this node.
		 * 
		 * should only be used by MainServer. 
		 * 
		 * \return void
		 */
		void register_servers();

		/**
		 * \brief registers a new child with this node
		 * 
		 * Adds child at the end of the child name list (or at the given index), and attempts to create a branch plug for the child. If the plugin fails to load the function will return false and
		 * 	an error message will be place in the error_msg member.
		 * 
		 * 
		 * \return true if the operation worked false if a failure occured 
		 */
		bool add_child(std::shared_ptr<BranchSocketType> socket_ptr, std::string child_name, std::string plug_type="dhtt_plugins::PtrBranchPlug", int index=-1);

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
		 * \return true if child was activated successfully 
		 */
		bool async_activate_child(std::string child_name, dhtt_msgs::action::Activation::Goal activation_goal);

		/**
		 * \brief propogates maintain conditions request to given child
		 * 
		 * \param child_name name of child to request
		 * \param condition_goal goal to send child
		 * 
		 * \return true if maintenance request was sent correctly
		 */
		bool async_request_conditions(std::string child_name, dhtt_msgs::action::Condition::Goal condition_goal);

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
		 * \brief propogates condition maintenance to children
		 * 
		 * sends a Condition action to all children, and counts the number of expected responses for the block function.
		 * 
		 * \return void
		 */
		void request_conditions_from_children();

		/**
		 * \brief busy waits until the number of responses equals the number of activations sent out
		 * 
		 * number of activations is incremented with each call of async_activate_child, number of responses is counted in the store_result_callback
		 * 
		 * \return void
		 */
		bool block_for_activation_from_children();

		/**
		 * \brief busy waits until all condition responses are received from children
		 * 
		 * \return void
		 */
		bool block_for_conditions_from_children();

		/**
		 * \brief getter for the responses from children
		 * 
		 * Should only be called after block_for_responses_from_children. 
		 * 
		 * \return map of child_names to the response they sent back after activating
		 */
		std::map<std::string, dhtt_msgs::action::Activation::Result> get_activation_results(); 

		/**
		 * \brief getter for the condition responses from children
		 * 
		 * \return map of child_names to their corresponding pre and post conditions
		 */
		std::map<std::string, dhtt_msgs::action::Condition::Result> get_condition_results();

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
		 * \brief notifies any knowledge sensitive nodes that there have been changes to the param server
		 * 
		 * For some nodes new knowledge can change how they should behave (for instance a find action will change destination to the correct one if the object is found when looking for something else).
		 * 	This publishes an empty message on the /updated_knowledge topic that will then utilize the callbacks in GOiTRs to have them update their own state.
		 * 
		 * \return void
		 */
		void fire_knowledge_updated();

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

		/**
		 * \brief activates the auction mechanism of task nodes and behavior nodes
		 * 
		 * The major functionality of the activate function is described in the paper on dHTT's extensively. Essentially, this function runs the internal logic of the NodeType plugin for this 
		 * 	node. It performs the following steps:
		 * - if the node is active it runs the auction_callback of the NodeType plugin and then collects and sends back the request 
		 * - if the node is working it runs the work_callback of the NodeType plugin and then releases and passes the appropriate resources
		 * 
		 * \param goal_handle internal action_server representation of the current goal
		 */
		dhtt_msgs::action::Activation::Result activate(dhtt_msgs::action::Activation::Goal goal);

		/**
		 * \brief Sends a request to the children to get pre and postconditions, then combines them based on logic in the node_type and saves and returns the result
		 */
		dhtt_msgs::action::Condition::Result combine_child_conditions(dhtt_msgs::action::Condition::Goal goal);

		/**
		 * \brief debugging tool to print the resources that this tree thinks are canonical
		 * 
		 * \param compare vector to print alongside the available resources for comparison
		 * 
		 * \return void
		 */
		void print_resources(std::vector<dhtt_msgs::msg::Resource> compare);

		/**
		 * \brief Sets the changed flag to maintain on the next call to maintain pre/postconditions
		 * 
		 * \param n_val value to the set the internal flag to
		 * 
		 * \return void
		 * 
		 */
		void set_changed_flag(bool n_val);

		/**
		 * \brief sets internal flag to maint pre/postconditions on child on next call
		 * 
		 * \param child name of child to change flag for
		 * \param n_val value to set the internal flag to
		 * 
		 * \return void
		 */
		void set_child_changed(std::string child, bool n_val);

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
		// rclcpp_action::GoalResponse goal_activation_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const dhtt_msgs::action::Activation::Goal> goal);

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
		// rclcpp_action::CancelResponse cancel_activation_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle);

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
		// void activation_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle);

		/**
		 * \brief accepts any goal for now
		 */
		// rclcpp_action::GoalResponse condition_goal_activation_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const dhtt_msgs::action::Condition::Goal> goal);

		/**
		 * \brief cancels any goal for now
		 */
		// rclcpp_action::CancelResponse condition_cancel_activation_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Condition>> goal_handle);

		/**
		 * \brief if the request is just a GET returns the pre and postconditions, if it is a maintain request this spins up a thread to run the combine_child_contions function instead.
		 */
		// void condition_activation_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Condition>> goal_handle);


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
		// void store_result_callback( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Activation>::WrappedResult & result, std::string node_name );

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
		// void store_failed_callback( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Activation>::WrappedResult & result, std::string node_name );

		/**
		 * \brief stores the conditions of the given child
		 * 
		 * \param result from child
		 * \param node_name child who sent back result
		 * 
		 * \return void
		 */
		// void store_condition_callback ( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Condition>::WrappedResult & result, std::string node_name );


		// service callbacks

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
		 * Checks key value pairs stored by the NodeType against those on the param server. If they are the same returns True.
		 * 
		 * \return True if preconditions are met
		 */
		bool check_preconditions();

		/**
		 * \brief applies postconditions to world state
		 * 
		 * Postconditions are defined in nthe node type of this class. This changes the parameter server to reflect this behavior's postconditions.
		 * 
		 * \return void
		 */
		void apply_postconditions();

		/**
		 * \brief calculates activation potential as a function of the estimate from the NodeType plugin
		 * 
		 * \deprecated
		 * 
		 * \return activation_potential of the node
		 */
		double calculate_activation_potential();

		/**
		 * \brief update the currently available resources with the resources list on the parameter server
		 * 
		 * \return void
		 */
		void pull_resources();

		// callback group
		rclcpp::CallbackGroup::SharedPtr conc_group;
		rclcpp::SubscriptionOptions sub_opts;
		rclcpp::PublisherOptions pub_opts;

		// publishers 
		rclcpp::Publisher<dhtt_msgs::msg::Node>::SharedPtr status_pub;

		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr knowledge_pub;

		// subscribers
		rclcpp::Subscription<dhtt_msgs::msg::Resources>::SharedPtr resources_sub;

		// communication_aggregator_ptr
		std::shared_ptr<CommunicationAggregator> global_com;

		// members
		pluginlib::ClassLoader<NodeType> node_type_loader;
		pluginlib::ClassLoader<GoitrType> goitr_type_loader;
		pluginlib::ClassLoader<BranchSocketType> branch_socket_type_loader;
		pluginlib::ClassLoader<BranchPlugType> branch_plug_type_loader;
		pluginlib::ClassLoader<PotentialType> potential_type_loader;

		std::shared_ptr<NodeType> logic;
		std::shared_ptr<GoitrType> replanner;
		std::shared_ptr<BranchSocketType> parent_communication_socket;
		std::map<std::string, std::pair<bool, std::shared_ptr<BranchPlugType>>> child_communication_plugs;
		std::shared_ptr<PotentialType> potential;

		dhtt_msgs::msg::NodeStatus status;
		std::map<std::string, dhtt_msgs::action::Activation::Result::SharedPtr> responses;
		std::map<std::string, dhtt_msgs::action::Condition::Result::SharedPtr> child_conditions;

		std::mutex logic_mut;
		std::mutex maintenance_mut;

		std::condition_variable resource_condition;

		std::vector<dhtt_msgs::msg::Resource> owned_resources;
		std::vector<dhtt_msgs::msg::Resource> available_resources;
		std::vector<dhtt_msgs::msg::Resource> passed_resources;

		std::vector<dhtt_msgs::msg::Resource> subtree_owned_resources;

		std::vector<std::string> child_names;
		std::map<std::string, bool> child_changed;

		std::string name;
		std::string parent_name;
		std::string plugin_name;
		std::string socket_name;
		std::string goitr_name;

		std::string active_child_name;

		double activation_potential;

		int priority;
		int resources_owned_by_subtree;

		std::atomic_bool resource_status_updated;
		bool has_goitr;
		bool first_activation;
		bool active;
		bool subtree_has_changed;

		bool successful_load;
		std::string error_msg;

	private:

		void add_unique_resources(std::vector<dhtt_msgs::msg::Resource> to_count); 
		void remove_unique_resources(std::vector<dhtt_msgs::msg::Resource> to_count);
	};

}

#endif // NODE_HPP