#include "pddl_template_receiver.hpp"
#include <fstream>
#include <algorithm>

namespace robot_planner {

static void replace_all(std::string & data, const std::string & from, const std::string & to)
{
  size_t start_pos = 0;
  while ((start_pos = data.find(from, start_pos)) != std::string::npos) {
    data.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}

PddlTemplateReceiver::PddlTemplateReceiver(const rclcpp::NodeOptions & options)
: Node("pddl_template_receiver", options)
{
  problem_name_ = this->declare_parameter<std::string>("problem_name", "generated_problem");
  domain_name_ = this->declare_parameter<std::string>("domain_name", "generated_domain");
  robot_cur_zone_ = this->declare_parameter<std::string>("robot_cur_zone", "zone_1");
  robot_goal_zone_ = this->declare_parameter<std::string>("robot_goal_zone", "zone_2");
  output_file_ = this->declare_parameter<std::string>("output_file", "pddl/generated_domain.pddl");
  template_topic_ = this->declare_parameter<std::string>("domain_template_topic", "pddl_template");

  template_sub_ = this->create_subscription<std_msgs::msg::String>(
    template_topic_, 10,
    std::bind(&PddlTemplateReceiver::template_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "PddlTemplateReceiver node started.");
}

void PddlTemplateReceiver::template_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string filled = replace_placeholders(msg->data);
  save_to_file(filled);
  RCLCPP_INFO(this->get_logger(), "Received PDDL template and saved to %s", output_file_.c_str());
}

std::string PddlTemplateReceiver::replace_placeholders(const std::string & tpl) const
{
  std::string result = tpl;
  replace_all(result, "PROBLEM_NAME", problem_name_);
  replace_all(result, "DOMAIN_NAME", domain_name_);
  replace_all(result, "ROBOT_CUR_ZONE", robot_cur_zone_);
  replace_all(result, "ROBOT_GOAL_ZONE", robot_goal_zone_);
  return result;
}

void PddlTemplateReceiver::save_to_file(const std::string & pddl) const
{
  std::ofstream out(output_file_);
  if (!out) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", output_file_.c_str());
    return;
  }
  out << pddl;
  out.close();
}

}  // namespace robot_planner

