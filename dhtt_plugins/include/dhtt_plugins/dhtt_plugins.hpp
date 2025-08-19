#ifndef DHTT_PLUGINS__CHTT_PLUGINS_HPP_
#define DHTT_PLUGINS__CHTT_PLUGINS_HPP_

#include "dhtt_plugins/visibility_control.h"

// list of plugin files to include here
#include "dhtt_plugins/behaviors/test/test_behavior.hpp"
#include "dhtt_plugins/behaviors/test/mock_move_behavior.hpp"
#include "dhtt_plugins/behaviors/test/mock_look_behavior.hpp"
#include "dhtt_plugins/tasks/root_behavior.hpp"
#include "dhtt_plugins/tasks/and_behavior.hpp"
#include "dhtt_plugins/tasks/then_behavior.hpp"
#include "dhtt_plugins/tasks/or_behavior.hpp"

#include "dhtt_plugins/behaviors/action_type.hpp"
#include "dhtt_plugins/behaviors/move_behavior.hpp"
#include "dhtt_plugins/behaviors/pick_behavior.hpp"
#include "dhtt_plugins/behaviors/place_behavior.hpp"

#include "dhtt_plugins/goitrs/goitr_test.hpp"
#include "dhtt_plugins/goitrs/find_goitr.hpp"

#include "dhtt_plugins/tree/ptr_branch.hpp"

#include <pluginlib/class_list_macros.hpp>

// NodeTypes
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::TestBehavior, dhtt::NodeType)
// PLUGINLIB_EXPORT_CLASS(dhtt_plugins::MockMoveBehavior, dhtt::NodeType)
// PLUGINLIB_EXPORT_CLASS(dhtt_plugins::MockLookBehavior, dhtt::NodeType)

PLUGINLIB_EXPORT_CLASS(dhtt_plugins::RootBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::ThenBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::AndBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::OrBehavior, dhtt::NodeType)

PLUGINLIB_EXPORT_CLASS(dhtt_plugins::MoveBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::PickBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::PlaceBehavior, dhtt::NodeType)

// GoitrTypes
// PLUGINLIB_EXPORT_CLASS(dhtt_plugins::GoitrTest, dhtt::GoitrType)
// PLUGINLIB_EXPORT_CLASS(dhtt_plugins::FindGoitr, dhtt::GoitrType)

// BranchTypes
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::PtrBranchSocket, dhtt::BranchSocketType)
PLUGINLIB_EXPORT_CLASS(dhtt_plugins::PtrBranchPlug, dhtt::BranchPlugType)

#endif  // DHTT_PLUGINS__HTT_PLUGINS_HPP_
