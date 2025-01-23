#ifndef DHTT_REORDER_HPP
#define DHTT_REORDER_HPP

#include "rclcpp/rclcpp.hpp"

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/node_type.hpp"
#include "dhtt_plugins/behaviors/action_type.hpp"
#include "std_msgs/msg/string.hpp"

namespace dhtt_reorder
{
    class PickBehavior : public dhtt_plugins::ActionType
    {
    public:
        /**
         * \brief parses parameters for the PickBehavior: activation potential, and object label
         *
         * The params for this behavior have the following meaning:
         * 		- activation_potential: given activation potential value for the behavior. This is useful for getting consistent behavior orderings during testing.
         * 		- object: label of the object to pick up. This should be on the experiment_moveit server after running a separate behavior for scanning object locations.
         *
         * \param params vector of string parameters from the yaml description
         *
         * \return void
         */
        void parse_params(std::vector<std::string> params) override;

        /**
         * \brief Picks the object specified
         *
         * Sends a command to the experiment_moveit server, which commands the arm
         *
         * \param container a pointer to the node which is running this logic.
         *
         * \return void
         */
        void do_work(dhtt::Node* container) override;

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
         * \brief returns a list with only the gripper resource granted to this behavior
         *
         * The postcondition of the pick behavior is that the robot is now holding an object which means that the gripper resource should be retained.
         *
         * \param container a pointer to the node which is running this logic.
         *
         * \return a list with the gripper resource holding the object which was picked
         */
        std::vector<dhtt_msgs::msg::Resource> get_retained_resources(dhtt::Node* container) override;

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
        std::vector<dhtt_msgs::msg::Resource> get_released_resources(dhtt::Node* container) override;

        /**
         * \brief callback for receiving done from the experiment_moveit server
         *
         * Recieves a notification that the moveit interface finished successfully or not
         *
         * \param data message received over the bridge
         *
         * \return void
         */
        void done_callback(std::shared_ptr<std_msgs::msg::String> data);

    protected:
        bool work_done;

        double activation_potential;
        std::string object_name;
    };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(dhtt_reorder::PickBehavior, dhtt::NodeType)

#endif // DHTT_REORDER_HPP
