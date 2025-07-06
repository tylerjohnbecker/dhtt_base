#ifndef THEN_BEHAVIOR_HPP
#define THEN_BEHAVIOR_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"

#include <vector>
#include <algorithm>
#include <string>

namespace dhtt_plugins
{
	/**
	 * \brief Implementation of the THEN node for dHTTs
	 * 
	 * A THEN node has to run all of it's children in a specified order. The order is currently taken from the literal order that they are listed
	 * 	in the yaml description file.
	 */
	class ThenBehavior : public dhtt::NodeType
	{
	public:
		void initialize(std::vector<std::string> params) override;

		/**
		 * \brief auction behavior for the THEN subtask
		 * 
		 * The then node always activates all of the children which haven't finished in order to calculate the activation potential, however, only the request
		 * 	of the next child in the queue is sent up the tree. Passed resources are given immediately to children for consideration when creating their request.
		 * 
		 * \param container see dhtt::NodeType
		 * 
		 * \return ActivationResult for the Node class' activation action server 
		 */
		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;

		/**
		 * \brief work behavior of the THEN subtask
		 * 
		 * The THEN simply passes the success down to the next child in the queue. This callback is also important as the passed resources after the child is done 
		 * 	are saved to the container.
		 * 
		 * \param container see dhtt::NodeType
		 * 
		 * \return ActivationResult for the Node class' activation action server 
		 */
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container ) override;

		/**
		 * \brief the THEN node logically and's it's child preconditions and then removes any temporal post->preconditions relationships that would be internally satisfied
		 */
		void maintain_conditions(dhtt::Node* container) override;

		/**
		 * \brief this behavior takes no parameters
		 */
		void parse_params( std::vector<std::string> params ) override;

		/**
		 * \brief gives the activation potential of this subtask
		 * 
		 * Activation Potential for an THEN node is calculated as the average of all children which have not finished execution.
		 * 
		 * \return activation potential of this subtask 
		 */
		double get_perceived_efficiency(dhtt::Node* container) override;

		/**
		 * \brief this behavior is done when all children are done.
		 */
		bool is_done() override;

	protected:
		std::mutex queue_index_mut;

		double activation_potential;

		int child_queue_index;
		int child_queue_size;
		int next;
		bool started_activation;
		bool created;
	private:
	};
}

#endif //THEN_BEHAVIOR_HPP