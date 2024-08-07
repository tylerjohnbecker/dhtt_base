#ifndef SUB_SERVER_HPP_
#define SUB_SERVER_HPP_

// cpp includes
#include <string.h>
#include <vector>
#include <memory>
#include <experimental/filesystem>

// ros2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// dhtt message includes
#include "dhtt_msgs/msg/node.hpp"
#include "dhtt_msgs/msg/pair.hpp"
#include "dhtt_msgs/msg/update_info.hpp"
#include "dhtt_msgs/msg/result.hpp"
#include "dhtt_msgs/srv/fetch_info.hpp"
#include "dhtt_msgs/srv/fetch_request.hpp"
#include "dhtt_msgs/srv/modify_request.hpp"
#include "dhtt_msgs/srv/control_request.hpp"
#include "dhtt_msgs/srv/goitr_request.hpp"

namespace dhtt
{

	/**
	 * \brief sub server for a subtree in the dHTT
	 * 
	 * This class is intended to be used by GOiTRs or other similar constructs in order to communicate with the main server and give
	 * 	direct changes to the subtree. A parent GOiTR for instance will use this class as an interface with which it can replan it's
	 * 	subtree.
	 */ 

	class SubServer : public rclcpp::Node
	{
	public:

		friend class GoitrType;

		/**
		 * \brief base constructor for the SubServer class.
		 * 
		 * Initializes all connections with the MainServer. Also creates it's own executor and therefore can run single threaded commands.
		 * 
		 * \param node_name name of the node at the top of this sub server's subtree
		 * 
		 * \return void
		 */
		SubServer( std::string node_name , std::string subtree_filename, std::vector<std::string> file_args );

		~SubServer();

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

		/**
		 * \brief builds the subtree that serves as the base for this action
		 * 
		 * Each goitr represents some high level action. The main idea is for each member of the tree to be left high level until the first activation. At that
		 * 	point the goitr should construct its own subtree and then activation should continue. For now this will be constant and the subtree will just be loaded
		 *  in from a file, however, it might be better to consider this more dynamically and already have done some planning here.
		 * 
		 * \return void 
		 */
		bool build_subtree();

		std::string main_server_topic, node_name; 
		std::vector<std::string> child_node_names;

		std::map<std::string, std::shared_future<dhtt_msgs::srv::ModifyRequest::Response::SharedPtr>> futures_to_process;
		std::map<std::string, bool> future_complete;

	private:
		bool modify( const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response );
		bool fetch( const std::shared_ptr<dhtt_msgs::srv::FetchRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::FetchRequest::Response> response );

		void update_to_fit_request(const std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response> response);

		rclcpp::Client<dhtt_msgs::srv::ModifyRequest>::SharedPtr modify_client;
		rclcpp::Client<dhtt_msgs::srv::FetchRequest>::SharedPtr fetch_client;
		rclcpp::Client<dhtt_msgs::srv::ControlRequest>::SharedPtr control_client;

		rclcpp::Service<dhtt_msgs::srv::GoitrRequest>::SharedPtr parent_service;

		rclcpp::Publisher<dhtt_msgs::msg::Result>::SharedPtr result_pub;

		std::shared_ptr<std::thread> service_thread;

		std::string filename;
		std::vector<std::string> args;

		bool thread_running;

	};

}

#endif // SUB_SERVER_HPP_