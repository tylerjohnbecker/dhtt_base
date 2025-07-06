#ifndef ROOT_BEHAVIOR_HPP
#define ROOT_BEHAVIOR_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/exceptions.h"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"

#include "dhtt_msgs/msg/resource.hpp"
#include "dhtt_msgs/msg/resources.hpp"

#include "dhtt_msgs/srv/internal_control_request.hpp"

#include <vector>
#include <string>

namespace dhtt_plugins
{

	/**
	 * \brief implementation of the dHTT root node 
	 * 
	 * The root node is always the root of a dHTT and cannot be placed elsewhere. It is a special node in the sense that it starts the auction and manages the state
	 * 	of the resources available for the behaviors. The root node also publishes the resource state to every node in the tree which is information they node to 
	 * 	properly make requests.
	 */
	class RootBehavior : public dhtt::NodeType
	{
	public:
		
		/**
		 * \brief initializes the major topics and a personal executor for the root node
		 * 
		 * starts a status publisher for itself, a control request server for communication with MainServer, an executor which is used to publish resources, and spins a separate thread
		 * 	for managing asynchronous callbacks from requests.
		 * 
		 * \param params see parse_params for what should go in here
		 * 
		 * \return void
		 */
		void initialize(std::vector<std::string> params) override;

		/**
		 * \brief callback for auction behavior of a root node
		 * 
		 * All of the work that a root node does is contained in this callback. The root node starts by spreading activation and waiting for all responses, then it clears the request and
		 * 	spreads activation again. Then after getting another response it releases the appropriate resources and starts again. This process repeats until the direct child of the root node
		 * 	is done at which point the tree stops running and the root node returns out of the auction callback.
		 * 
		 * \param container see dhtt::NodeType
		 * 
		 * \return ActivationResult to let the MainServer know the tree is done executing
		 */
		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;

		/**
		 * \brief empty callback
		 * 
		 * Work is not needed for the root node because it is already being done in the auction.
		 * 
		 * \param container see dhtt::NodeType
		 * 
		 * \return ActivationResult which is empty because this callback is not used.
		 */
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container ) override;

		/**
		 * \brief parses the params for the root node
		 * 
		 * root node expects exactly one param in the format "key: val". The key should be "path" and the value is the path to the robot resources file (e.g. in dhtt/robots). This file contains 
		 * 	the list of all resources on the robot which can be used by behaviors as well as any specific rules that those resources might have.
		 * 
		 * \param params see dhtt::NodeType
		 * 
		 * \return void
		 */
		void parse_params( std::vector<std::string> params ) override;

		/**
		 * \brief get activation potential for the root node
		 * 
		 * activation potential for the root node is meaningless since it cannot do work so this just returns a default value of 1.0
		 * 
		 * \return 1.0
		 */
		double get_perceived_efficiency(dhtt::Node* container) override;

		/**
		 * \brief gets whether the root node is done
		 * 
		 * The root done is only done once all of the nodes in the tree are done or more simply when it's immediate child is done.
		 * 
		 * \return True if the tree is finished executing, False otherwise
		 */
		bool is_done() override;

	protected:

		/**
		 * \brief loads the robot resources in the robot_resources_path
		 * 
		 * Loads the resources from the given file into the list of resources available to the behaviors. The resources file must be formatted similar to the one in dhtt/robots but most importantly 
		 * 	each resource needs to have a name and a type. This function can throw an uncaught yaml exception if the file is incorrect which will stop the MainServer from starting.
		 * 
		 * \return void
		 */
		void load_resources_from_yaml();

		/**
		 * \brief publishes the current state of the resources to the rest of the dHTT
		 * 
		 * tasks the list of resources and puts it on the resource state topic so that the behavior nodes can accurately assess the possibility of their request
		 * 
		 * \return void
		 */
		void publish_resources();

		/**
		 * \brief fullfills a request from the child
		 * 
		 * updates the status of the resources on the robot such that a given set of resources has been handed out. Resources are given on a first found basis so for example if there are two arms on the
		 * 	robot and the left is listed first in the yaml it will always be utilized before the right arm by default. 
		 * 
		 * \param to_give list of resources which need their status updated (given with types)
		 * 
		 * \return list of granted resources (given with canonical names)
		 */
		std::vector<dhtt_msgs::msg::Resource> give_resources(std::vector<dhtt_msgs::msg::Resource> to_give);

		/**
		 * \brief releases resources that were in use
		 * 
		 * After behaviors are done running they generally release at least some resources so that the root node can hand them out again. This function searches the internal list for the resources by name
		 * 	and releases them.  
		 *
		 * \param to_release the list of resources to release
		 * 
		 * \return void
		 */
		void release_resources(std::vector<dhtt_msgs::msg::Resource> to_release);

		/**
		 * \brief releases all resources
		 * 
		 * This method serves to clean up in case there are resources still in use after the tree has stopped running. Just goes resource by resource and releases them.
		 * 
		 * \return void
		 */
		void release_all_resources();

		/**
		 * \brief callback to handle stop requests from MainServer
		 * 
		 * Currently only graceful stops can be handled but essentially when this service is called the next time a behavior finishes executing the auction will not continue until the MainServer does it explicitly.
		 * 
		 * \param request an InternalControlRequest
		 * \param response an InternalControlResponse
		 * 
		 * \return void
		 */
		void control_callback( const std::shared_ptr<dhtt_msgs::srv::InternalControlRequest::Request> request, std::shared_ptr<dhtt_msgs::srv::InternalControlRequest::Response> response );

		/**
		 * \brief just a helpful spinning method
		 * 
		 * This node needs the extra spinning thread to ensure that it can send out the messages for the status and that it can asynchronously receive a control request from MainServer
		 * 
		 * \return void
		 */
		void async_spin();

		std::string robot_resources_file_path;

		std::vector<dhtt_msgs::msg::Resource> canonical_resources_list;
		std::string robot_name;

		bool children_done;
		bool interrupted;
		bool slow;
		
		std::shared_ptr<rclcpp::Node> pub_node_ptr;
		std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> resource_executor;
		
		rclcpp::Publisher<dhtt_msgs::msg::Resources>::SharedPtr status_pub;
		rclcpp::Service<dhtt_msgs::srv::InternalControlRequest>::SharedPtr control_server;

		std::shared_ptr<std::thread> spin_thread;

	private:
	};
}

#endif //ROOT_BEHAVIOR_HPP