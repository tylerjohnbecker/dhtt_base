#ifndef DHTT_PLUGINS__CHTT_PLUGINS_HPP_
#define DHTT_PLUGINS__CHTT_PLUGINS_HPP_

#include "dhtt_plugins/visibility_control.h"

// list of plugin files to include here
#include "dhtt_plugins/behaviors/test_behavior.hpp"
#include "dhtt_plugins/tasks/root_behavior.hpp"
#include "dhtt_plugins/tasks/then_behavior.hpp"
#include "dhtt_plugins/tasks/or_behavior.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(dhtt_plugins::TestBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::RootBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::ThenBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::OrBehavior, dhtt::NodeType)

#endif  // DHTT_PLUGINS__HTT_PLUGINS_HPP_
