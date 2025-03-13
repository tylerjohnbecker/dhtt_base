#include "dhtt_plugins/behaviors/cooking_object_exists_behavior.hpp"

namespace dhtt_plugins
{
double CookingObjectExistsBehavior::get_perceived_efficiency()
{
	return CookingBehavior::get_perceived_efficiency();
}

void CookingObjectExistsBehavior::do_work(dhtt::Node *container)
{
	this->done = this->destination_is_good;
}

std::vector<dhtt_msgs::msg::Resource>
CookingObjectExistsBehavior::get_retained_resources(dhtt::Node *container)
{
	return container->get_owned_resources();
}
std::vector<dhtt_msgs::msg::Resource>
CookingObjectExistsBehavior::get_released_resources(dhtt::Node *container)
{
	return {};
}

std::vector<dhtt_msgs::msg::Resource> CookingObjectExistsBehavior::get_necessary_resources()
{
	return {};
}
} // namespace dhtt_plugins