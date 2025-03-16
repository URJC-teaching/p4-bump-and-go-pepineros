#ifndef BUMP_BUMPGO__BUMPGO_BEHAVIOR_HPP_
#define BUMP_BUMPGO__BUMPGO_BEHAVIOR_HPP_

// 📌 Se incluyen las bibliotecas necesarias
#include "geometry_msgs/msg/twist.hpp"  // Para enviar comandos de velocidad
#include "rclcpp/rclcpp.hpp"  // Librería base de ROS 2 para crear nodos
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"  // Para detectar eventos del bumper (choques)

namespace bumpgo  // 📌 Definimos el namespace del nodo
{

using namespace std::chrono_literals;  // 📌 Permite usar "2s" en lugar de definir duraciones manualmente

// 📌 Definimos la clase BumpGoBehavior, que hereda de rclcpp::Node (un nodo en ROS 2)
class BumpGoBehavior : public rclcpp::Node
{
public:
  BumpGoBehavior();  // 📌 Constructor de la clase (configura el nodo)

private:
  // 📌 Función que se ejecuta cuando se detecta un choque con el bumper
  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);

  // 📌 Función principal del robot, se ejecuta en bucle para actualizar su comportamiento
  void control_cycle();

  // 📌 Definimos los estados posibles del robot
  static const int FORWARD = 0;  // Estado de avanzar
  static const int AVOID = 1;    // Estado de evitar (retroceder)
  static const int TURN = 2;     // Estado de giro

  int state_;  // 📌 Variable que almacena el estado actual del robot
  rclcpp::Time state_ts_;  // 📌 Marca de tiempo del último cambio de estado

  // 📌 Función para cambiar el estado del robot
  void go_state(int new_state);

  // 📌 Funciones que verifican si es necesario cambiar de estado
  bool check_forward_2_avoid();  // Si detecta un choque, pasa de FORWARD a AVOID
  bool check_avoid_2_turn();  // Si termina de retroceder, pasa a TURN
  bool check_turn_2_forward();  // Si termina de girar, pasa a FORWARD

  // 📌 Duraciones de cada estado
  const rclcpp::Duration AVOID_TIME {2s};  // Tiempo que retrocede en AVOID
  const rclcpp::Duration TURN_TIME {2s};   // Tiempo que gira en TURN

  // 📌 Velocidades del robot
  static constexpr float SPEED_LINEAR = 0.3;  // Velocidad de avance y retroceso
  static constexpr float SPEED_ANGULAR = 1.0;  // Velocidad de giro

  // 📌 Elementos de ROS 2 para interactuar con el robot
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;  // 📌 Suscripción al bumper
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;  // 📌 Publicador de velocidades en /cmd_vel
  rclcpp::TimerBase::SharedPtr timer_;  // 📌 Temporizador para ejecutar `control_cycle()`

  // 📌 Último evento del bumper registrado
  kobuki_ros_interfaces::msg::BumperEvent::UniquePtr last_bump_;
};

}  // namespace bumpgo

#endif  // BUMP_BUMPGO__BUMPGO_BEHAVIOR_HPP_
