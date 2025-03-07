#include "dhtt_plugins/behaviors/cooking_execute_behavior.hpp"

namespace dhtt_plugins
{
void CookingExecuteBehavior::parse_params(std::vector<std::string> params) {}

double CookingExecuteBehavior::get_perceived_efficiency() {}

void CookingExecuteBehavior::do_work(dhtt::Node *container) {}

std::vector<dhtt_msgs::msg::Resource>
CookingExecuteBehavior::get_retained_resources(dhtt::Node *container)
{
}

std::vector<dhtt_msgs::msg::Resource>
CookingExecuteBehavior::get_released_resources(dhtt::Node *container)
{
}
} // namespace dhtt_plugins