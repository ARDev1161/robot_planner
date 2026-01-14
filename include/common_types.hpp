#ifndef ROBOT_PLANNER__COMMON_TYPES_HPP_
#define ROBOT_PLANNER__COMMON_TYPES_HPP_

#include <string>
#include <map>

/// @file common_types.hpp
/// @brief Common type definitions shared across the robot_planner package.

namespace robot_planner {

/// @brief Description of a single symbolic action used throughout the planner.
struct Action
{
  /// Name of the action
  std::string name;
  /// Key-value map with action parameters
  std::map<std::string, std::string> params;
};

}  // namespace robot_planner

#endif  // ROBOT_PLANNER__COMMON_TYPES_HPP_
