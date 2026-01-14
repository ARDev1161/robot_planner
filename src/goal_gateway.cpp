#include <algorithm>
#include <cctype>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace robot_planner {

class GoalGateway : public rclcpp::Node
{
public:
  GoalGateway()
  : Node("goal_gateway")
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/robot_planner/goal");
    status_topic_ = declare_parameter<std::string>("status_topic", "/robot_planner/goal_status");
    set_goal_service_ = declare_parameter<std::string>(
      "set_goal_service", "/problem_expert/add_problem_goal");
    allow_parallel_ = declare_parameter<bool>("allow_parallel", false);

    goal_sub_ = create_subscription<std_msgs::msg::String>(
      goal_topic_, 10, std::bind(&GoalGateway::goalCallback, this, std::placeholders::_1));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
  }

private:
  static std::string trim(std::string input)
  {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    input.erase(input.begin(), std::find_if(input.begin(), input.end(), not_space));
    input.erase(std::find_if(input.rbegin(), input.rend(), not_space).base(), input.end());
    return input;
  }

  void publishStatus(const std::string &status, const std::string &detail)
  {
    if (!status_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = status;
    if (!detail.empty()) {
      msg.data += ": " + detail;
    }
    status_pub_->publish(msg);
  }

  void goalCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string goal = trim(msg->data);
    if (goal.empty()) {
      RCLCPP_WARN(get_logger(), "Goal request is empty.");
      publishStatus("rejected", "empty goal");
      return;
    }

    if (in_flight_ && !allow_parallel_) {
      RCLCPP_WARN(get_logger(), "Goal request ignored: another goal is in flight.");
      publishStatus("rejected", "busy");
      return;
    }

    if (!problem_expert_client_) {
      problem_expert_client_ = std::make_shared<plansys2::ProblemExpertClient>();
    }

    in_flight_ = true;
    publishStatus("submitted", goal);

    bool ok = true;
    if (!allow_parallel_) {
      ok = problem_expert_client_->clearGoal();
    }
    if (ok) {
      try {
        const auto goal_tree = parser::pddl::fromString(goal);
        ok = problem_expert_client_->setGoal(goal_tree);
      } catch (const std::exception &ex) {
        RCLCPP_WARN(get_logger(), "Goal parse failed: %s", ex.what());
        ok = false;
      }
    }
    in_flight_ = false;
    if (ok) {
      RCLCPP_INFO(get_logger(), "Goal accepted: %s", goal.c_str());
      publishStatus("accepted", goal);
    } else {
      RCLCPP_WARN(get_logger(), "Goal rejected");
      publishStatus("rejected", "setGoal failed");
    }
  }

  std::string goal_topic_;
  std::string status_topic_;
  std::string set_goal_service_;
  bool allow_parallel_{false};
  bool in_flight_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_client_;
};

}  // namespace robot_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_planner::GoalGateway>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
