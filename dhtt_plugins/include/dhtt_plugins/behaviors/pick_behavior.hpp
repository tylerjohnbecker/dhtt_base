#ifndef PICK_BEHAVIOR_HPP
#define PICK_BEHAVIOR_HPP

#include "dhtt_plugins/behaviors/action_type.hpp"
#include "std_msgs/msg/string.hpp"

namespace dhtt_plugins
{

	/**
	 * \brief Action behavior to pick up the first object detected in front of the robot at the current position
	 */
	class PickBehavior : public ActionType
	{
	public:
		/**
		 * \brief parses parameters for the PickBehavior: activation potential, and object label
		 * 
		 * The params for this behavior have the following meaning:
		 * 		- activation_potential: given activation potential value for the behavior. This is useful for getting consistent behavior orderings during testing.
		 * 		- object: label of the object to pick up.
		 * 
		 * \param params vector of string parameters from the yaml description
		 * 
		 * \return void
		 */
		void parse_params(std::vector<std::string> params) override;

		/**
		 * \brief Picks the object specified
		 * 
		 * For the purposes of this experiment the robot runs on ROS1, so this function passes a command message over ROS1 bridge to a server which picks the object.
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
		double get_perceived_efficiency(dhtt::Node* container) override;

		/**
		 * \brief returns a list with only the gripper resource granted to this behavior
		 * 
		 * The postcondition of the pick behavior is that the robot is now holding an object which means that the gripper resource should be retained.
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return a list with the gripper resource holding the object which was picked
		 */
		std::vector<dhtt_msgs::msg::Resource> get_retained_resources( dhtt::Node* container ) override;

		/**
		 * \brief releases all resources except the gripper
		 * 
		 * This behavior only guarantees that a object is now held and cannot be undone until the object is placed. Therefore all other resources than the gripper
		 * 	are free to be used by other behaviors. 
		 * 
		 * \param container a pointer to the node which is running this logic.
		 * 
		 * \return a list with all resources but the gripper
		 */
		std::vector<dhtt_msgs::msg::Resource> get_released_resources( dhtt::Node* container ) override; 

		/**
		 * \brief this behavior needs the move base of the robot and an arm to grab the object
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
		void done_callback( std::shared_ptr<std_msgs::msg::String> data );

	protected:
		bool work_done;

		double activation_potential;
		std::string object_name;
	};

}

#endif // PICK_BEHAVIOR_HPP