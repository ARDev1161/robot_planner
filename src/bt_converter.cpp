#include "robot_planner/bt_converter.hpp"
#include <sstream>

namespace robot_planner {

BTConverter::BTConverter()
{
}

BTConverter::~BTConverter()
{
}

std::string BTConverter::convertPlanToXML(const std::vector<Action>& plan)
{
  std::ostringstream xml_stream;
  xml_stream << "<root main_tree_to_execute=\"MainTree\"><BehaviorTree ID=\"MainTree\">";
  for (const auto & action : plan)
  {
    if (action.name == "move")
    {
      xml_stream << "<Move target_room=\"" << action.params.at("target_room") << "\"/>";
    }
    else if (action.name == "pick-up")
    {
      xml_stream << "<PickUp object=\"" << action.params.at("object") << "\"/>";
    }
    // Добавьте обработку других действий по мере необходимости.
  }
  xml_stream << "</BehaviorTree></root>";
  return xml_stream.str();
}

}  // namespace robot_planner
