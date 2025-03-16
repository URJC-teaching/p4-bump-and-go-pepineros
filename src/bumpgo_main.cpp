#include <memory>
#include "bumpgo/bumpgo_behavior.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto bumpgo_node = std::make_shared<bumpgo::BumpGoBehavior>();

  rclcpp::spin(bumpgo_node);  // Ejecutar el nodo en un solo hilo

  rclcpp::shutdown();
  return 0;
}
