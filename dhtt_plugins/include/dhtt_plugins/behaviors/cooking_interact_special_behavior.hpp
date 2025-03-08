#ifndef COOKING_INTERACT_SPECIAL_HPP
#define COOKING_INTERACT_SPECIAL_HPP

#include "dhtt_plugins/behaviors/cooking_behavior.hpp"

namespace dhtt_plugins
{
/**
 * Essentially the same thing as CookingPickBehavior, except allows the agent to pick off an object
 * from a plate instead of taking the entire plate stack.
 */
class CookingInteractSpecialBehavior : public CookingBehavior
{
  public:
	/**
	 * @return 1.0 if agent is right next to the object. 0.0 otherwise.
	 */
	double get_perceived_efficiency() override;

	/**
	 * First move_to the target location (to set the orientation), then pick
	 * @param container unused
	 */
	void do_work(dhtt::Node *container) override;

	/**
	 * Retains the gripper used to pick the object
	 * @param container pointer to the node which is running this logic.
	 * @return vector taking ownership of one of the robot arms
	 */
	std::vector<dhtt_msgs::msg::Resource> get_retained_resources(dhtt::Node *container) override;

	/**
	 * Releases all resources except the gripper that is now holding the object
	 * @param container pointer to the node which is running this logic.
	 * @return vector releasing the owned robot arm
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

#endif // COOKING_INTERACT_SPECIAL_HPP
