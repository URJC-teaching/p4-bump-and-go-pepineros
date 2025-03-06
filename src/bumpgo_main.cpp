#include <memory>
#include "bumpgo/bumpgo_behavior.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 3);

  auto bumpgo_node = std::make_shared<bumpgo::BumpGoBehavior>();

  exe.add_node(bumpgo_node->get_node_base_interface());

  bumpgo_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  bumpgo_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
