#ifndef MOVE_BEHAVIOR_HPP
#define MOVE_BEHAVIOR_HPP

#include "dhtt_plugins/behaviors/action_type.hpp"
#include "std_msgs/msg/string.hpp"

namespace dhtt_plugins
{
	/**
	 * \brief Moves to a semantically specified location on a given map
	 */
	class MoveBehavior : public ActionType
	{
	public:
		/**
		 * \brief parses activation_potential and destination parameters for the move behavior
		 * 
		 * The meaning of each parameter is as follows:
		 *		- activation_potential: a given activation potential for the behavior (for testing specific behavior orders)
		 * 		- destination: a named position on the param server (will not complete work if this does not exist)
		 * 
		 * \param params a vector of string params parsed from the yaml description
		 * 
		 * \return void
		 */
		void parse_params(std::vector<std::string> params) override;

		/**
		 * \brief moves the robot to the destination
		 * 
		 * For the experimental implementation of the dHTT paper this behavior sent a message over ROS1 bridge meaning just a string was sent to a different server.
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return void
		 */
		void do_work( dhtt::Node* container ) override;

		/**
		 * \brief calculates activation potential for the move behavior
		 * 
		 * For the experimental implementation consistent behavior orderings were desired so this function returns the value specified in the yaml description.
		 * 
		 * \return activation_potential of the node
		 */
		double get_perceived_efficiency() override;

		/**
		 * \brief retains all resources
		 * 
		 * Move behaviors are expected to have some meaning therefore resources are kept. For example, if a move behavior is before some other behavior it is expected that
		 * 	there will be some task to perform at the destination and therefore the resources should be kept to prevent the robot from getting distracted.
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return the current list of all owned resources
		 */
		std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) override;

		/**
		 * \brief releases no resources 
		 * 
		 * As described in the retained_resources function nothing should be released after moving to a destination to prevent the robot from getting distracted.
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return an empty vector of resources 
		 */
		std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node* container ) override; 

		/**
		 * \brief this behavior needs the move base of the robot
		 */
		virtual std::vector<dhtt_msgs::msg::Resource> get_necessary_resources() override;

		/**
		 * \brief callback for receiving done from the ROS1 implementation of MoveBehavior
		 * 
		 * For this experiment the actual robot specific implementation had to be done in ROS1 so this callback receives a notification that movement finished over ROS1 bridge.
		 * 
		 * \param data message received over the bridge
		 * 
		 * \return void
		 */
		void done_callback( const std::shared_ptr<std_msgs::msg::String> data );

	protected:
		bool work_done;

		double activation_potential;
		std::string destination;
	};

}

#endif // MOVE_BEHAVIOR_HPP