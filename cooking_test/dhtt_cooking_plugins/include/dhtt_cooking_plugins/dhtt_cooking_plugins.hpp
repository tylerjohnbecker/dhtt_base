#ifndef DHTT_COOKING_PLUGINS__CHTT_PLUGINS_HPP_
#define DHTT_COOKING_PLUGINS__CHTT_PLUGINS_HPP_

#include "dhtt_cooking_plugins/visibility_control.h"

// list of plugin files to include here
#include "dhtt_cooking_plugins/behaviors/cooking_execute_behavior.hpp"
#include "dhtt_cooking_plugins/behaviors/cooking_interact_special_behavior.hpp"
#include "dhtt_cooking_plugins/behaviors/cooking_move_behavior.hpp"
#include "dhtt_cooking_plugins/behaviors/cooking_pick_behavior.hpp"
#include "dhtt_cooking_plugins/behaviors/cooking_plate_place_behavior.hpp"
#include "dhtt_cooking_plugins/behaviors/cooking_object_exists_behavior.hpp"
#include "dhtt_cooking_plugins/behaviors/cooking_place_behavior.hpp"
#include "dhtt_cooking_plugins/behaviors/defer_behavior.hpp"

#include <pluginlib/class_list_macros.hpp>

// NodeTypes
PLUGINLIB_EXPORT_CLASS(dhtt_cooking_plugins::CookingExecuteBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_cooking_plugins::CookingInteractSpecialBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_cooking_plugins::CookingMoveBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_cooking_plugins::CookingPickBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_cooking_plugins::CookingPlaceBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_cooking_plugins::CookingPlatePlaceBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_cooking_plugins::CookingObjectExistsBehavior, dhtt::NodeType)
PLUGINLIB_EXPORT_CLASS(dhtt_cooking_plugins::DeferBehavior, dhtt::NodeType)

#endif  // DHTT_COOKING_PLUGINS__CHTT_PLUGINS_HPP_
