#include <memory>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/// @file drop_action_node.cpp
/// @brief Plansys2 action node that drops a carried object.

class DropAction : public plansys2::ActionExecutorClient
{
public:
  DropAction()
  : plansys2::ActionExecutorClient("drop", 250ms),
    progress_(0.0)
  {}

private:
  void do_work() override
  {
    // Имитация процесса отпускания объекта
    if (progress_ < 1.0) {
      progress_ += 0.05;
      send_feedback(progress_, "Dropping object...");
    } else {
      finish(true, 1.0, "Drop completed successfully");
    }
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DropAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "drop"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
