#include "dhtt_plugins/behaviors/cooking_pick_behavior.hpp"

namespace dhtt_plugins
{
void CookingPickBehavior::parse_params(std::vector<std::string> params) {}

double CookingPickBehavior::get_perceived_efficiency() {}

void CookingPickBehavior::do_work(dhtt::Node *container) {}

std::vector<dhtt_msgs::msg::Resource>
CookingPickBehavior::get_retained_resources(dhtt::Node *container)
{
}

std::vector<dhtt_msgs::msg::Resource>
CookingPickBehavior::get_released_resources(dhtt::Node *container)
{
}
} // namespace dhtt_plugins