#ifndef MAIN_SERVER
#define MAIN_SERVER

// cpp includes
#include <string.h>
#include <iostream>

// yaml-cpp includes
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/exceptions.h"

// boost includes
#include "boost/thread/mutex.hpp"

// ros2 includes
#include "rclcpp/rclcpp.hpp"

// dhtt includes

// dhtt message includes
#include "dhtt_msgs/msg/node.hpp"
#include "dhtt_msgs/msg/node_status.hpp"
#include "dhtt_msgs/msg/resource.hpp"
#include "dhtt_msgs/msg/subtree.hpp"

// dhtt srv includes
#include "dhtt_msgs/srv/control_request.hpp"
#include "dhtt_msgs/srv/fetch_request.hpp"
#include "dhtt_msgs/srv/modify_request.hpp"
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

namespace dHTT
{
	class MainServer : public rclcpp::Node
	{
	public:
		MainServer(std::string node_name);
		~MainServer();

	protected:

	private:

		/// *** PRIVATE MEMBER FUNCTIONS ***
		// server callbacks
		void modify_callback( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response );
		void control_callback( const std::shared_ptr<dhtt_msgs::srv::ControlRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ControlRequest::Response> response );
		void fetch_callback( const std::shared_ptr<dhtt_msgs::srv::FetchRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> response );
		void register_callback( const std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration::Response> response );

		// modify helpers
		std::string add_node( std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string parent_name, dhtt_msgs::msg::Node to_add );
		std::string add_nodes_from_file( std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string parent_name, std::string file_name );
		std::string remove_node( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response, std::string to_remove );
		std::string change_params( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request );

		// control helpers
		std::string stop_tree( bool interrupt );
		std::string start_tree();
		std::string save_tree();
		std::string reset_tree();

		// fetch helpers
		std::vector<dhtt_msgs::msg::Subtree> fetch_subtrees_by_common_name( std::string name );
		std::vector<dhtt_msgs::msg::Subtree> fetch_subtrees_by_type( int type );
		dhtt_msgs::msg::Subtree fetch_subtree_by_name( std::string name );

		// subscriber callbacks
		void status_callback( const std::shared_ptr<dhtt_msgs::msg::Node> data );

		// generally helpful functions
		void maintain_local_subtree();
		void fill_subtree_metrics( dhtt_msgs::msg::Subtree& to_fill );

		dhtt_msgs::msg::Node get_active_behavior_node();
		std::vector<dhtt_msgs::msg::Node> get_active_nodes();

		dhtt_msgs::msg::Subtree construct_subtree_from_node_iter( std::vector<dhtt_msgs::msg::Node>::iterator top_node );

		/// *** PRIVATE MEMBERS ***
		// external services
		rclcpp::Service<dhtt_msgs::srv::ModifyRequest>::SharedPtr modify_server;
		rclcpp::Service<dhtt_msgs::srv::ControlRequest>::SharedPtr control_server;
		rclcpp::Service<dhtt_msgs::srv::FetchRequest>::SharedPtr fetch_server;

		// internal services
		rclcpp::Service<dhtt_msgs::srv::InternalServiceRegistration>::SharedPtr registration_server;

		// internal service clients
		std::map<std::string, rclcpp::Client<dhtt_msgs::srv::InternalControlRequest>::SharedPtr> tree_control_service;
		std::map<std::string, rclcpp::Client<dhtt_msgs::srv::InternalModifyRequest>::SharedPtr> tree_modify_service;

		// publishers
		rclcpp::Publisher<dhtt_msgs::msg::NodeStatus>::SharedPtr root_status_pub;

		// internal subscribers
		rclcpp::Subscription<dhtt_msgs::msg::Node>::SharedPtr status_sub;

		// mutexes
		boost::mutex modify_mut;

		// members
		std::map<std::string, rclcpp::Node> ros2_node_list; //////////////////// change to dhtt_node as soon as that exists for now i don't even need to maintain it
		dhtt_msgs::msg::Subtree node_list; 

		int total_nodes_added;
		bool verbose;

	};

}

#endif //MAIN_SERVER 