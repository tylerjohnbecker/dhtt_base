#ifndef DHTT_PLUGINS__CHTT_PLUGINS_HPP_
#define DHTT_PLUGINS__CHTT_PLUGINS_HPP_

#include "dhtt_plugins/visibility_control.h"

// list of plugin files to include here
#include "dhtt_plugins/behaviors/dummy_behavior.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(dhtt_plugins::DummyBehavior, dhtt::NodeType)

#endif  // DHTT_PLUGINS__HTT_PLUGINS_HPP_
