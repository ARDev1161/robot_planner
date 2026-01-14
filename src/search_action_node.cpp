#include <memory>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/// @file search_action_node.cpp
/// @brief Action node that simulates searching for a target.

class SearchAction : public plansys2::ActionExecutorClient
{
public:
  SearchAction()
  : plansys2::ActionExecutorClient("search", 250ms),
    progress_(0.0)
  {}

private:
  void do_work() override
  {
    // Имитация поиска цели в заданной локации
    if (progress_ < 1.0) {
      progress_ += 0.05;
      send_feedback(progress_, "Searching in location...");
    } else {
      finish(true, 1.0, "Search completed successfully");
    }
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SearchAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "search"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
