#ifndef FIND_GOITR_HPP_
#define FIND_GOITR_HPP_

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/planning/goitr_type.hpp"

namespace dhtt_plugins
{

	/**
	 * \brief GOiTR to systematically search the given set of locations for an object
	 * 
	 * This GOiTR is meant to attach to a THEN node. It build a subtree consisting of just a move behavior and a lookfor behavior.
	 * 	Initially, the most likely location for finding the object will be chosen and then each location will be checked. If the move
	 * 	fails to complete it is simply run again for now, and if the look behavior fails then a new location is chosen and the THEN node 
	 * 	is reset. If the object is found as a side effect of another look behavior somewhere in the tree the move destination will be updated.
	 * 	A parent GOiTR also has the ability to change the target obh
	 */
	class FindGoitr : public dhtt::GoitrType
	{
	protected:

		/**
		 * \brief not sure what this needs to do differently yet
		 */
		virtual void init_derived(std::string node_name, std::vector<std::string> params) override;

		/**
		 * \brief not sure what this needs to do differently yet
		 */
		virtual void destruct_derived() override;

		/**
		 * \brief enables a parent GOiTR to change the target of this subtask
		 * 
		 * just provides an interface with interacting with this node. Perhaps this could request status updates from this subtask but for now it just enables
		 * 	the parent to change the target without interfering with the subtree.
		 */
		virtual void parent_service_callback(std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Request> req, std::shared_ptr<dhtt_msgs::srv::GoitrRequest::Response> res) override;

		/**
		 * \brief if the location of the target object is found changes the move destination
		 */
		virtual void knowledge_update_callback(/*whatever message for this*/) override;

		/**
		 * \brief main callback of this node
		 * 
		 * When the move behavior finishes it should always be successful in a static environment, but if it fails we should just run it again. When the look
		 * 	for behavior finishes if it succeeds then we are done as the object is found, and if it fails we should reset the THEN node that the find goitr is 
		 * 	attached to and change the destination of the move behavior. Also the move behavior should have it's status modified to waiting once again.
		 */
		virtual void child_finished_callback(/*whatever message for this*/) override;

	private:

	}

}

#endif // FIND_GOITR_HPP_