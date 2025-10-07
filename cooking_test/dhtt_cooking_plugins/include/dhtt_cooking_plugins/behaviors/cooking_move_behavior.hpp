#ifndef COOKING_MOVE_BEHAVIOR_HPP
#define COOKING_MOVE_BEHAVIOR_HPP

#include "dhtt_cooking_plugins/behaviors/cooking_behavior.hpp"

namespace dhtt_cooking_plugins
{
class CookingMoveBehavior : public CookingBehavior
{
  public:
	/**
	 * @return inverse of 1 + the l2 distance to the target destination. That is, 1 if we are
	 * already at the destination, or approaching 0 when far away.
	 *
	 * TODO Tyler uses parameter this->should_unmark to set priority for certain move_to behaviors
	 *  (doubles activation).
	 */
	double get_perceived_efficiency(dhtt::Node* container) override;

	/**
	 * Makes a move_to request to the target destination.
	 *
	 * If the object is not marked yet (i.e. it is not marked on the paramserver but we have
	 * param 'mark: ...') we will taint it with this->destination_mark.
	 *
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

	/**
	 * \brief needs a counter/dispenser/tool to run
	 */
	void populate_resource_lists(dhtt::Node* container) override;

  protected:
  private:
};
} // namespace dhtt_cooking_plugins

#endif // COOKING_MOVE_BEHAVIOR_HPP
