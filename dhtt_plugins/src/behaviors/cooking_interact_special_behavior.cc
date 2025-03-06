#include "dhtt_plugins/behaviors/cooking_interact_special_behavior.hpp"

namespace dhtt_plugins
{
void CookingInteractSpecialBehavior::parse_params(std::vector<std::string> params) {}

double CookingInteractSpecialBehavior::get_perceived_efficiency() {}

void CookingInteractSpecialBehavior::do_work(dhtt::Node *container) {}

std::vector<dhtt_msgs::msg::Resource>
CookingInteractSpecialBehavior::get_retained_resources(dhtt::Node *container)
{
}

std::vector<dhtt_msgs::msg::Resource>
CookingInteractSpecialBehavior::get_released_resources(dhtt::Node *container)
{
}
} // namespace dhtt_plugins