#ifndef COOKING_PLACE_BEHAVIOR_HPP
#define COOKING_PLACE_BEHAVIOR_HPP

#include "dhtt_cooking_plugins/behaviors/cooking_behavior.hpp"

namespace dhtt_cooking_plugins
{
/**
 * Essentially the same as CookingPickBehavior, except the resource management is different since we
 * no longer need the gripper.
 */
class CookingPlaceBehavior : public CookingBehavior
{
  public:
	/**
	 * @return 1.0 if agent is right next to the object. 0.0 otherwise.
	 */
	double get_perceived_efficiency(dhtt::Node* container) override;

	/**
	 * First move_to the target location (to set the orientation), then place.
	 *
	 * If placing on a deliversquare, send a NOP so it clears.
	 *
	 * In the special case that we are placing on a deliversquare that is marked with our taint,
	 * unmark it to free it up for other behaviors. Normally the pick behavior is the only one to
	 * unmark static objects, but deliversquares 'pick' themselves.
	 *
	 * If param "unmark" was passed, unmark the object
	 *
	 * @param container unused
	 */
	void do_work(dhtt::Node *container) override;

	/**
	 * Retains nothing
	 * @param container unused
	 * @return empty vector
	 */
	std::vector<dhtt_msgs::msg::Resource> get_retained_resources(dhtt::Node *container) override;

	/**
	 * Needs one gripper and the move base (to set the agent's orientation)
	 * @return vector containing a gripper resource and base
	 */
	std::vector<dhtt_msgs::msg::Resource> get_necessary_resources() override;

  protected:
  private:
};
} // namespace dhtt_cooking_plugins

#endif // COOKING_PLACE_BEHAVIOR_HPP
