#ifndef COOKING_EXECUTE_BEHAVIOR_HPP
#define COOKING_EXECUTE_BEHAVIOR_HPP

#include "dhtt_plugins/behaviors/cooking_behavior.hpp"

namespace dhtt_plugins
{
class CookingExecuteBehavior : public CookingBehavior
{
  public:
	/**
	 * @return 1.0 if agent is right next to the object. 0.0 otherwise.
	 */
	double get_perceived_efficiency() override;

	/**
	 * First move_to the target location (to set the orientation), then execute
	 * @param container unused
	 */
	void do_work(dhtt::Node *container) override;

	/**
	 * CookingExecuteBehavior uses one arm and the base to execute on an object, then immediately
	 * releases the arm, so we retain nothing.
	 * @param container pointer to the node which is running this logic.
	 * @return empty vector
	 */
	std::vector<dhtt_msgs::msg::Resource> get_retained_resources(dhtt::Node *container) override;

	/**
	 * CookingExecuteBehavior uses one arm and the base to execute on an object, then immediately
	 * releases the arm, which is to say, all owned resources.
	 * @param container pointer to the node which is running this logic.
	 * @return vector releasing all owned resources
	 */
	std::vector<dhtt_msgs::msg::Resource> get_released_resources(dhtt::Node *container) override;

	/**
	 * Needs one gripper and the move base (to set the agent's orientation)
	 * @return vector containing a gripper resource and base
	 */
	std::vector<dhtt_msgs::msg::Resource> get_necessary_resources() override;

  protected:
  private:
};
} // namespace dhtt_plugins

#endif // COOKING_EXECUTE_BEHAVIOR_HPP
