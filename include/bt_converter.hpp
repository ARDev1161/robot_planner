#ifndef ROBOT_PLANNER__BT_CONVERTER_HPP_
#define ROBOT_PLANNER__BT_CONVERTER_HPP_

#include <string>
#include <vector>
#include <map>
#include "common_types.hpp"

/// @file bt_converter.hpp
/// @brief Utilities for converting a plan into a Behavior Tree description.

namespace robot_planner {

/// @brief Helper class that produces a Behavior Tree XML description from a plan.
class BTConverter
{
public:
  /// Default constructor
  BTConverter() = default;
  /// Default destructor
  ~BTConverter() = default;

  /// Convert a plan to the XML representation used by BehaviorTree.CPP
  /// @param plan Sequence of actions to convert
  /// @return XML string describing the behavior tree
  std::string convertPlanToXML(const std::vector<Action> & plan);
};

}  // namespace robot_planner

#endif  // ROBOT_PLANNER__BT_CONVERTER_HPP_
