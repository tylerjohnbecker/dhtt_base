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

#include "dhtt_msgs/action/activation.hpp"

#include "dhtt/tree/node_type.hpp"

#define TREE_PREFIX "/dhtt"
#define REGISTER_CHILD_POSTFIX "/register_child"
#define ACTIVATION_POSTFIX "/activate"
#define RESOURCES_POSTFIX "/resource"

namespace dhtt
{
	class NodeType;

	class Node : public rclcpp::Node
	{
	public:
		Node(std::string name, std::string type, std::vector<std::string> params, std::string parent_name);
		~Node();

		bool loaded_successfully();
		std::string get_error_msg();

		std::vector<dhtt_msgs::msg::Resource> get_owned_resources();
		void set_owned_resources(std::vector<dhtt_msgs::msg::Resource> set_to);

		void register_with_parent();
		bool remove_child(std::string child_name);

		// helpers for nodetype
		void async_activate_child(std::string child_name, dhtt_msgs::action::Activation::Goal activation_goal);
		void activate_all_children(dhtt_msgs::action::Activation::Goal activation_goal);
		bool block_for_responses_from_children();
		std::map<std::string, dhtt_msgs::action::Activation::Result::SharedPtr> get_activation_results(); 

		std::vector<std::string> get_child_names();
		std::string get_active_child_name();

		bool isRequestPossible(std::vector<dhtt_msgs::msg::Resource> requested_resources);

		void update_status( int8_t n_state );

	protected:

		// activation action server callbacks
		rclcpp_action::GoalResponse goal_activation_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const dhtt_msgs::action::Activation::Goal> goal);
		rclcpp_action::CancelResponse cancel_activation_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle);
		void activation_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle);

		void store_result_callback( const rclcpp_action::ClientGoalHandle<dhtt_msgs::action::Activation>::WrappedResult & result, std::string node_name );

		void activate(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dhtt_msgs::action::Activation>> goal_handle);

		// service callbacks
		void register_child_callback(std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Response> response);

		// subscriber callbacks
		void resource_availability_callback( const dhtt_msgs::msg::Resources::SharedPtr canonical_list );

		// helpful member functions
		bool check_preconditions();
		double calculate_activation_potential();
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

		std::vector<dhtt_msgs::msg::Resource> owned_resources;
		std::vector<dhtt_msgs::msg::Resource> available_resources;
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

		bool successful_load;
		std::string error_msg;

	private:
	};

}

#endif // NODE_HPP