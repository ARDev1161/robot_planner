#include <memory>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class WaitAction : public plansys2::ActionExecutorClient
{
public:
  WaitAction()
  : plansys2::ActionExecutorClient("wait", 250ms),
    progress_(0.0)
  {}

private:
  void do_work() override
  {
    // Имитация ожидания на месте
    if (progress_ < 1.0) {
      progress_ += 0.03;
      send_feedback(progress_, "Waiting...");
    } else {
      finish(true, 1.0, "Wait completed successfully");
    }
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaitAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "wait"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
