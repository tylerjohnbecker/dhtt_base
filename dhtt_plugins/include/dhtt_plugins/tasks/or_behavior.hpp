#ifndef OR_BEHAVIOR_HPP
#define OR_BEHAVIOR_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"

#include <vector>
#include <string>

namespace dhtt_plugins
{
	/**
	 * \brief Implementation of the OR node for dHTTs
	 * 
	 * The OR node will be done after a single one of it's children is done, and therefore it chooses the one with the highest
	 * 	activation potential.
	 */
	class OrBehavior : public dhtt::NodeType
	{
	public:

		void initialize(std::vector<std::string> params) override;

		/**
		 * \brief Auction behavior for the OR node
		 *	
		 * If this node has not already run one of it's children then it passes the request of the child with the highest activation potential. If a subtask
		 * 	has already been chosen once then it is the only child which is activated and it's requests are passed up until it is done.
		 * 
		 * \param container see dhtt::NodeType
		 * 
		 * \return ActivationResult for the Node class' activation action server 
		 */
		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;

		/**
		 * \brief Work behavior for the OR node
		 * 
		 * This node just spreads the success of the request to the active child. Essentially, after the child is picked the OR node becomes just a middleman
		 * 	and doesn't have much internal logic. 
		 * 
		 * \param container see dhtt::NodeType
		 * 
		 * \return ActivationResult for the Node class' activation action server 
		 */
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container ) override;

		/**
		 * \brief this behavior takes no parameters
		 */
		void parse_params( std::vector<std::string> params ) override;

		/**
		 * \brief gives the activation potential of this subtask
		 * 
		 * Activation Potential for an OR node is calculated as the highest activation potential of it's children if one hasn't already run, or the active child's
		 * 	activation potential otherwise.
		 * 
		 * \return activation potential of this subtask 
		 */
		double get_perceived_efficiency() override;

		/**
		 * \brief this behavior is done when a child is done.
		 */
		bool is_done() override;

	protected:
		bool has_chosen_child;
		bool child_has_run;
		bool child_done;
		double activation_potential;

		std::string activated_child_name;
	private:
	};
}

#endif //OR_BEHAVIOR_HPP