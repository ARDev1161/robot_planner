#ifndef ROBOT_PLANNER__BT_CONVERTER_HPP_
#define ROBOT_PLANNER__BT_CONVERTER_HPP_

#include <string>
#include <vector>
#include <map>

namespace robot_planner {

// Можно использовать ту же структуру Action, что и в plan_executor.hpp
struct Action {
  std::string name;
  std::map<std::string, std::string> params;
};

class BTConverter
{
public:
  BTConverter();
  ~BTConverter();

  // Преобразует план (вектор действий) в XML-строку для BT
  std::string convertPlanToXML(const std::vector<Action>& plan);
};

}  // namespace robot_planner

#endif  // ROBOT_PLANNER__BT_CONVERTER_HPP_
