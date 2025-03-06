#ifndef BUMP_BUMPGO__BUMPGO_BEHAVIOR_HPP_
#define BUMP_BUMPGO__BUMPGO_BEHAVIOR_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"  // Mantener esta librería
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"  // Incluir para el mensaje de bumper

namespace bumpgo
{
  // Cambio: heredar de rclcpp::Node en lugar de CascadeLifecycleNode
  class BumpGoBehavior : public rclcpp::Node
  {
  public:
    BumpGoBehavior();

  private:
    void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);
    void control_cycle();

    // Eliminado: Funciones de ciclo de vida extendido
    // on_activate y on_deactivate eliminadas

    static const int FORWARD = 0;
    static const int AVOID = 1;

    int state_;
    rclcpp::Time state_ts_;

    void go_state(int new_state);
    bool check_forward_2_avoid();
    bool check_avoid_2_forward();

    const rclcpp::Duration AVOID_TIME{4s};

    rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    kobuki_ros_interfaces::msg::BumperEvent::UniquePtr last_bump_;
  };

}  // namespace bumpgo

#endif  // BUMP_BUMPGO__BUMPGO_BEHAVIOR_HPP_
