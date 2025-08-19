#ifndef COOKING_PICK_BEHAVIOR_HPP
#define COOKING_PICK_BEHAVIOR_HPP

#include "dhtt_cooking_plugins/behaviors/cooking_behavior.hpp"

namespace dhtt_cooking_plugins
{
class CookingPickBehavior : public CookingBehavior
{
  public:
	/**
	 * @return 1.0 if agent is right next to the object. 0.0 otherwise.
	 */
	double get_perceived_efficiency(dhtt::Node* container) override;

	/**
	 * First move_to the target location (to set the orientation), then pick.
	 *
	 * If the object is not marked yet (i.e. it is not marked on the paramserver but we have
	 * param 'mark: ...') we will taint it with this->destination_mark.
	 *
	 * If the object we are picking is on top of a static object (it always should be) that is
	 * marked, unmark that static object to free up that counter/cutboard/dispenser for other
	 * behaviors.
	 *
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
} // namespace dhtt_cooking_plugins

#endif // COOKING_PICK_BEHAVIOR_HPP
