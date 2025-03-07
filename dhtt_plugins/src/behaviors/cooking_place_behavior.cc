#include "dhtt_plugins/behaviors/cooking_place_behavior.hpp"

namespace dhtt_plugins
{
void CookingPlaceBehavior::parse_params(std::vector<std::string> params) {}

double CookingPlaceBehavior::get_perceived_efficiency() {}

void CookingPlaceBehavior::do_work(dhtt::Node *container) {}

std::vector<dhtt_msgs::msg::Resource>
CookingPlaceBehavior::get_retained_resources(dhtt::Node *container)
{
}

std::vector<dhtt_msgs::msg::Resource>
CookingPlaceBehavior::get_released_resources(dhtt::Node *container)
{
}
} // namespace dhtt_plugins