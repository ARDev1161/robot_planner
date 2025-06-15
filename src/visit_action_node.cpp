#include <memory>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/// @file visit_action_node.cpp
/// @brief Action node that marks a location as visited.

class VisitAction : public plansys2::ActionExecutorClient
{
public:
  VisitAction()
  : plansys2::ActionExecutorClient("visit", 250ms),
    progress_(0.0)
  {}

private:
  void do_work() override
  {
    // Имитация процесса посещения локации
    if (progress_ < 1.0) {
      progress_ += 0.1;
      send_feedback(progress_, "Visiting location...");
    } else {
      finish(true, 1.0, "Visit completed successfully");
    }
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisitAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "visit"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
