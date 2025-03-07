#ifndef COOKING_PICK_BEHAVIOR_HPP
#define COOKING_PICK_BEHAVIOR_HPP

#include "dhtt_plugins/behaviors/action_type.hpp"

namespace dhtt_plugins
{
class CookingPickBehavior : public ActionType
{
  public:
	void parse_params(std::vector<std::string> params) override;
	double get_perceived_efficiency() override;
	void do_work(dhtt::Node *container) override;
	std::vector<dhtt_msgs::msg::Resource> get_retained_resources(dhtt::Node *container) override;
	std::vector<dhtt_msgs::msg::Resource> get_released_resources(dhtt::Node *container) override;
};
} // namespace dhtt_plugins

#endif // COOKING_PICK_BEHAVIOR_HPP
