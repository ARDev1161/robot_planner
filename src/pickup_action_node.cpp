#include <memory>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PickupAction : public plansys2::ActionExecutorClient
{
public:
  PickupAction()
  : plansys2::ActionExecutorClient("pick-up", 250ms),
    progress_(0.0)
  {}

private:
  void do_work() override
  {
    // Имитация процесса захвата объекта
    if (progress_ < 1.0) {
      progress_ += 0.05;
      send_feedback(progress_, "Picking up object...");
    } else {
      finish(true, 1.0, "Pickup completed successfully");
    }
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickupAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "pick-up"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
