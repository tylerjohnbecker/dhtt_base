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
	double get_perceived_efficiency() override;
	void do_work(dhtt::Node *container) override;
	std::vector<dhtt_msgs::msg::Resource> get_retained_resources(dhtt::Node *container) override;
	std::vector<dhtt_msgs::msg::Resource> get_released_resources(dhtt::Node *container) override;

  protected:
	double activation_potential;

  private:
};
} // namespace dhtt_plugins

#endif // COOKING_MOVE_BEHAVIOR_HPP
