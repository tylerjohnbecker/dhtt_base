#ifndef COOKING_OBJECT_EXISTS_BEHAVIOR_HPP
#define COOKING_OBJECT_EXISTS_BEHAVIOR_HPP

#include "dhtt_plugins/behaviors/cooking_behavior.hpp"

namespace dhtt_plugins
{
/**
 * Behavior intended to block a THEN node or trigger an OR node until an object with specified
 * conditions exists in the world.
 */
class CookingObjectExistsBehavior : public CookingBehavior
{
  public:
	/**
	 * @return 1 if the object exists, 0 otherwise.
	 */
	double get_perceived_efficiency(dhtt::Node* container) override;

	/**
	 * Fail if the object does not exist. Succeed if it does.
	 * @param container unused
	 */
	void do_work(dhtt::Node *container) override;

	/**
	 * @param container pointer to the node which is running this logic.
	 * @return Does not change the resources already owned by this node.
	 */
	std::vector<dhtt_msgs::msg::Resource> get_retained_resources(dhtt::Node *container) override;

	/**
	 * @param container pointer to the node which is running this logic.
	 * @return Does not change the resources already owned by this node.
	 */
	std::vector<dhtt_msgs::msg::Resource> get_released_resources(dhtt::Node *container) override;

	/**
	 * @return No resources needed
	 */
	std::vector<dhtt_msgs::msg::Resource> get_necessary_resources() override;

  protected:
  private:
};
} // namespace dhtt_plugins

#endif // COOKING_OBJECT_EXISTS_BEHAVIOR_HPP
