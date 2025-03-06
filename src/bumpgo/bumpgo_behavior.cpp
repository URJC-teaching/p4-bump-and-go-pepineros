#include "bumpgo/bumpgo_behavior.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bumpgo
{
  using namespace std::chrono_literals;
  using std::placeholders::_1;

  BumpGoBehavior::BumpGoBehavior()
  : rclcpp::Node("bumpgo_behavior"),  // Cambio: CascadeLifecycleNode -> rclcpp::Node
    state_(FORWARD)
  {
    bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
      "/events/bumper", 10, std::bind(&BumpGoBehavior::bumper_callback, this, _1));

    // Iniciar el temporizador para el ciclo de control
    timer_ = create_wall_timer(50ms, std::bind(&BumpGoBehavior::control_cycle, this));

    state_ts_ = now();

    last_bump_ = std::make_unique<kobuki_ros_interfaces::msg::BumperEvent>();
    last_bump_->state = kobuki_ros_interfaces::msg::BumperEvent::RELEASED;

    go_state(FORWARD);
  }

  void BumpGoBehavior::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
  {
    last_bump_ = std::move(msg);
  }

  void BumpGoBehavior::control_cycle()
  {
    switch (state_) {
      case FORWARD:
        RCLCPP_INFO(get_logger(), "BumpGo Behavior: FORWARD");

        if (check_forward_2_avoid()) {
          go_state(AVOID);
        }
        break;

      case AVOID:
        RCLCPP_INFO(get_logger(), "BumpGo Behavior: AVOID");

        if (check_avoid_2_forward()) {
          go_state(FORWARD);
        }
        break;
    }
  }

  void BumpGoBehavior::go_state(int new_state)
  {
    state_ = new_state;
    state_ts_ = now();
  }

  bool BumpGoBehavior::check_forward_2_avoid()
  {
    return last_bump_->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED;
  }

  bool BumpGoBehavior::check_avoid_2_forward()
  {
    return (now() - state_ts_) > AVOID_TIME;
  }

}  // namespace bumpgo
