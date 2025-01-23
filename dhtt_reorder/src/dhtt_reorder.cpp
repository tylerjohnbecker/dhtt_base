#include "dhtt_reorder/dhtt_reorder.hpp"

namespace dhtt_reorder
{
void PickBehavior::parse_params(std::vector<std::string> params)
{
}

void PickBehavior::do_work(dhtt::Node *container)
{
}

double PickBehavior::get_perceived_efficiency()
{
}

std::vector<dhtt_msgs::msg::Resource> PickBehavior::get_retained_resources(
    dhtt::Node *container)
{
}

std::vector<dhtt_msgs::msg::Resource> PickBehavior::get_released_resources(
    dhtt::Node *container)
{
}

void PickBehavior::done_callback(std::shared_ptr<std_msgs::msg::String> data)
{
}
}
