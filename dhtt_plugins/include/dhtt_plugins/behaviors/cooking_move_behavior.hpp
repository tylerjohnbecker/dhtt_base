#ifndef COOKING_MOVE_BEHAVIOR_HPP
#define COOKING_MOVE_BEHAVIOR_HPP

#include "dhtt_msgs/msg/cooking_observation.hpp"
#include "dhtt_msgs/srv/cooking_request.hpp"
#include "dhtt_plugins/behaviors/action_type.hpp"
#include "dhtt_plugins/behaviors/cooking_behavior.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dhtt_plugins
{
class CookingMoveBehavior : public CookingBehavior
{
  public:
	/**
	 * @return inverse of 1 + the l2 distance to the target destination. That is, 1 if we are
	 * already at the destination, or approaching 0 when far away.
	 */
	double get_perceived_efficiency() override;

	/**
	 * Makes a move_to request to the target destination.
	 * @param container unused
	 */
	void do_work(dhtt::Node *container) override;

	/**
	 * See comment in move_behavior.hpp
	 * @param container pointer to the node which is running this logic.
	 * @return Does not change the resources already owned by this node.
	 */
	std::vector<dhtt_msgs::msg::Resource> get_retained_resources(dhtt::Node *container) override;

	/**
	 * See comment in move_behavior.hpp
	 * @param container pointer to the node which is running this logic.
	 * @return Does not release any resources.
	 */
	std::vector<dhtt_msgs::msg::Resource> get_released_resources(dhtt::Node *container) override;

	/**
	 * Needs to own the base
	 * @return vector containing a base resource
	 */
	std::vector<dhtt_msgs::msg::Resource> get_necessary_resources() override;

  protected:
	double activation_potential;

  private:
};
} // namespace dhtt_plugins

#endif // COOKING_MOVE_BEHAVIOR_HPP
