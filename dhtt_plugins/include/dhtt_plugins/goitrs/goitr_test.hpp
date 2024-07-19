#ifndef GOITR_TEST
#define GOITR_TEST

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/server/goitr_type.hpp"

namespace dhtt_plugins 
{

	/**
	 * \brief GoitrTest is just for testing and has mostly blank functionality
	 * 
	 * This class is just meant for testing meaning it pretty much doesn't do anything. Parent service is utilized such that tests can call the service
	 * 	and this class simply passes the message to the MainServer. In other GOiTRS the class will utilize internal logic to create service calls.
	 */
	class GoitrTest : public dhtt::GoitrType
	{
	protected:

		/**
		 * \brief init for GOiTRTest. Just starts services for mainserver
		 * 
		 * This will evolve with the tests, but for now we just want to check that the Goitr starts correctly, dies correctly, and connects with the main server.
		 * 
		 * \param params a vector of params to initialize the node with (blank for testing)
		 * 
		 * \return void
		 */
		virtual void init (std::vector<std::string> params) override;

		/**
		 * \brief service available to tests to check functionality with calling MainServer
		 * 
		 * Currently this service can be used to add nodes, remove nodes, and change the parameters of nodes.
		 * 
		 * \param req Request from the parent
		 * \param res ptr to give back to the server
		 * 
		 * \return void
		 */
		virtual void parent_service(std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Request> req, std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Response> res) override;

		/**
		 * \brief Callback which defines the logic for when knowledge updates are received
		 * 
		 * empty
		 * 
		 * \param TBD
		 * 
		 * \return void
		 */
		virtual void knowledge_update_callback(/*whatever message for this*/) override;

		/**
		 * \brief Callback which define how the subtree should be built
		 * 
		 * empty
		 * 
		 * \param TBD
		 * 
		 * \return void
		 */
		virtual void first_activation_callback(/*whatever message for this*/) override;

		/**
		 * \brief Callback which defines how to act when a child that this node has direct access to has finished.0
		 * 
		 * empty
		 * 
		 * \param TBD
		 * 
		 * \return void
		 */
		virtual void child_finished_callback(/*whatever message for this*/) override;


	private:

	};

}

#endif // GOITR_TEST