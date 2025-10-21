#ifndef AND_BEHAVIOR_HPP
#define AND_BEHAVIOR_HPP

#include "dhtt_plugins/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"

#include <vector>
#include <string>

namespace dhtt_plugins
{
	/**
	 * \brief Implementation of the AND node for dHTTs
	 * 
	 * The AND node has no constraint on the ordering of the children and so it just runs all of them in decreasing order of activation potential.
	 * 	The AND node also has to run all of it's children.
	 */
	class AndBehavior : public dhtt::NodeType
	{
	public:

		void initialize(std::vector<std::string> params) override;

		/**
		 * \brief auction behavior for the AND node.
		 * 
		 * An AND node runs all of it's children in an unspecified order. The auction activates all children and collects their requests. Then, the request with the
		 * 	highest activation potential is chosen and sent up the tree. All other children are returned to the WAITING state. When the AND node is first activated
		 * 	it saves the resources that were passed and always passes those to the children. When each child is finished the AND adds the passed resources to a full
		 * 	list and then when it finishes passes the entire list of resources from the children.
		 * 
		 * \param container see dhtt::NodeType
		 * 
		 * \return ActivationResult for the Node class' activation action server 
		 */
		std::shared_ptr<dhtt_msgs::action::Activation::Result> auction_callback( dhtt::Node* container ) override;

		/**
		 * \brief work behavior for the AND node.
		 */
		std::shared_ptr<dhtt_msgs::action::Activation::Result> work_callback( dhtt::Node* container ) override;

		/**
		 * \brief the AND node performs a logical and of the predicates of its children for both pre and post conditions
		 */
		void maintain_conditions(dhtt::Node* container) override;

		/**
		 * \brief this behavior takes no parameters
		 */
		void parse_params( std::vector<std::string> params ) override;

		/**
		 * \brief gives the activation potential of this subtask
		 * 
		 * Activation Potential for an AND node is calculated as the average of the activation potential of it's children.
		 * 
		 * \return activation potential of this subtask 
		 */
		double get_perceived_efficiency(dhtt::Node* container) override;

		/**
		 * \brief this behavior is done when all children are done
		 */
		bool is_done() override;

	protected:

		/**
		 * \brief updates the total passed resources from an Activation result message
		 * 
		 * \param result used to update total
		 * 
		 * \return void
		 */
		void update_total_passed(dhtt_msgs::action::Activation::Result result);
	
		int num_active_children;
		double activation_potential;

		bool first_activation;

		std::vector<dhtt_msgs::msg::Resource> initially_passed;
		std::vector<dhtt_msgs::msg::Resource> total_passed;

	private:
	};
}

#endif //AND_BEHAVIOR_HPP