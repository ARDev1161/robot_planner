#ifndef ROBOT_PLANNER__PDDL_TEMPLATE_RECEIVER_HPP_
#define ROBOT_PLANNER__PDDL_TEMPLATE_RECEIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>

namespace robot_planner {

class PddlTemplateReceiver : public rclcpp::Node
{
public:
  explicit PddlTemplateReceiver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void template_callback(const std_msgs::msg::String::SharedPtr msg);
  std::string replace_placeholders(const std::string & tpl) const;
  void save_to_file(const std::string & pddl) const;

  std::string problem_name_;
  std::string domain_name_;
  std::string robot_cur_zone_;
  std::string robot_goal_zone_;
  std::string output_file_;
  std::string template_topic_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr template_sub_;
};

}  // namespace robot_planner

#endif  // ROBOT_PLANNER__PDDL_TEMPLATE_RECEIVER_HPP_
