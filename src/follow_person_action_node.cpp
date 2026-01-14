#include <memory>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

/// @file follow_person_action_node.cpp
/// @brief Plansys2 action node that follows a detected person using Nav2.

class FollowPersonAction : public plansys2::ActionExecutorClient
{
public:
  FollowPersonAction()
  : plansys2::ActionExecutorClient("follow_person", 250ms),
    progress_(0.0), goal_sent_(false), goal_finished_(false)
  {
    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");
  }

private:
  void do_work() override
  {
    // Имитация следования за человеком: предполагается, что координаты человека известны
    if (!goal_sent_) {
      if (!nav2_client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
        return;
      }

      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = now();
      // Пример целевых координат для следования (в реальности координаты будут обновляться)
      goal_msg.pose.pose.position.x = 3.0;
      goal_msg.pose.pose.position.y = 3.0;
      goal_msg.pose.pose.orientation.w = 1.0;

      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = [this](const auto & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          finish(true, 1.0, "Follow person completed successfully");
        } else {
          finish(false, progress_, "Follow person failed");
        }
        goal_finished_ = true;
      };

      nav2_client_->async_send_goal(goal_msg, send_goal_options);
      goal_sent_ = true;
      RCLCPP_INFO(get_logger(), "Nav2 goal sent for follow person action");
    }

    if (!goal_finished_) {
      if (progress_ < 0.99) {
        progress_ += 0.02;
      }
      send_feedback(progress_, "Following person...");
    }
  }

  float progress_;
  bool goal_sent_;
  bool goal_finished_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowPersonAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "follow_person"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
