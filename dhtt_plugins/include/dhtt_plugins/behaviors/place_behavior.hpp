#ifndef PLACE_BEHAVIOR_HPP
#define PLACE_BEHAVIOR_HPP

#include "dhtt_plugins/behaviors/action_type.hpp"
#include "std_msgs/msg/string.hpp"

namespace dhtt_plugins
{
	/**
	 * \brief Place behavior implementation
	 */
	class PlaceBehavior : public ActionType
	{
	public:
		/**
		 * \brief parses parameters for the PlaceBehavior: activation potential
		 * 
		 * The params for this behavior have the following meaning:
		 * 		- activation_potential: given activation potential value for the behavior. This is useful for getting consistent behavior orderings during testing.
		 * 
		 * \param params vector of string parameters from the yaml description
		 * 
		 * \return void
		 */
		void parse_params(std::vector<std::string> params) override;

		/**
		 * \brief Picks the object specified
		 * 
		 * For the purposes of this experiment the robot runs on ROS1, so this function passes a command message over ROS1 bridge to a server which places the object.
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return void
		 */
		void do_work( dhtt::Node* container ) override;

		/**
		 * \brief gives the activation potential of this behavior
		 * 
		 * The activation was set by a parameter for this experiment in order to ensure that the tasks would run in specific orders.
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return activation potential of the behavior
		 */
		double get_perceived_efficiency() override;

		/**
		 * \brief returns an empty list of resources 
		 * 
		 * The place behavior ensures that nothing is in the gripper after placing the object so there is no need to retain any resources after this behavior runs.
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return an empty vector of resources
		 */
		std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) override;

		/**
		 * \brief returns a list of all granted resources 
		 * 
		 * The place behavior ensures that nothing is in the gripper after placing the object so there is no need to retain any resources after this behavior runs.
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return a vector of all granted_resources to this behavior
		 */
		std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node* container ) override; 

		/**
		 * \brief callback for receiving done from the ROS1 implementation of MoveBehavior
		 * 
		 * For this experiment the actual robot specific implementation had to be done in ROS1 so this callback receives a notification that movement finished over ROS1 bridge.
		 * 
		 * \param data message received over the bridge
		 * 
		 * \return void
		 */
		void done_callback( std::shared_ptr<std_msgs::msg::String> data );

	protected:
		bool work_done;

		double activation_potential;
	};

}

#endif // PLACE_BEHAVIOR_HPP