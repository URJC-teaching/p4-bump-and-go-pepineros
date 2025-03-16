// 📌 Se incluyen las bibliotecas necesarias
#include "bumpgo/bumpgo_behavior.hpp"  // Incluye la declaración de la clase y las funciones
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"  // Para manejar eventos del bumper
#include "geometry_msgs/msg/twist.hpp"  // Para enviar comandos de velocidad
#include "rclcpp/rclcpp.hpp"  // Librería de ROS 2 para nodos y comunicación

namespace bumpgo  // 📌 Definimos el namespace del nodo
{
  using namespace std::chrono_literals;  // 📌 Permite usar "2s" en lugar de definir duraciones manualmente
  using std::placeholders::_1;  // 📌 Para usar `std::bind` en las funciones de callback

  // 📌 Constructor del nodo `BumpGoBehavior`
  BumpGoBehavior::BumpGoBehavior()
  : rclcpp::Node("bumpgo_behavior"),  // Inicializa el nodo con el nombre "bumpgo_behavior"
    state_(FORWARD)  // 📌 Se inicia en estado FORWARD (avanzando)
  {
    // 📌 Suscripción al bumper para detectar colisiones
    bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
      "/events/bumper", 10, std::bind(&BumpGoBehavior::bumper_callback, this, _1));

    // 📌 Publicador de velocidades en el topic "/cmd_vel"
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 📌 Se crea un temporizador que ejecuta `control_cycle()` cada 50ms
    timer_ = create_wall_timer(50ms, std::bind(&BumpGoBehavior::control_cycle, this));

    // 📌 Guarda la marca de tiempo actual
    state_ts_ = now();

    // 📌 Inicializa la variable del bumper sin detectar colisión
    last_bump_ = std::make_unique<kobuki_ros_interfaces::msg::BumperEvent>();
    last_bump_->state = kobuki_ros_interfaces::msg::BumperEvent::RELEASED;

    // 📌 Establece el estado inicial del robot como FORWARD
    go_state(FORWARD);
  }

  // 📌 Callback que se ejecuta cuando se detecta un evento del bumper
  void BumpGoBehavior::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
  {
    last_bump_ = std::move(msg);  // 📌 Guarda el nuevo estado del bumper
  }

  // 📌 Ciclo de control que decide el comportamiento del robot
  void BumpGoBehavior::control_cycle()
  {
    // 📌 Inicializa el mensaje de velocidad con valores en 0
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

    // 📌 Dependiendo del estado actual, se define la acción del robot
    switch (state_) {
        case FORWARD:  // 🚀 Estado de avance
            RCLCPP_INFO(get_logger(), "Estado: FORWARD - Avanzando");
            cmd_vel.linear.x = SPEED_LINEAR;  // 📌 Avanza a la velocidad definida

            if (check_forward_2_avoid()) {  // 📌 Si detecta un obstáculo, cambia a AVOID
                go_state(AVOID);
            }
            break;

        case AVOID:  // 🚀 Estado de evitar obstáculos (retroceder)
            RCLCPP_INFO(get_logger(), "Estado: AVOID - Retrocediendo");
            cmd_vel.linear.x = -SPEED_LINEAR;  // 📌 Retrocede a la misma velocidad

            if (check_avoid_2_turn()) {  // 📌 Si ya ha retrocedido suficiente tiempo, pasa a girar
                go_state(TURN);
            }
            break;

        case TURN:  // 🚀 Estado de giro
            RCLCPP_INFO(get_logger(), "Estado: TURN - Girando");
            cmd_vel.angular.z = SPEED_ANGULAR;  // 📌 Gira a la velocidad definida

            if (check_turn_2_forward()) {  // 📌 Si ya ha girado suficiente tiempo, vuelve a avanzar
                go_state(FORWARD);
            }
            break;
    }

    // 📌 Publica el comando de velocidad en `/cmd_vel`
    vel_pub_->publish(cmd_vel);  
    RCLCPP_INFO(get_logger(), "Publicando en /cmd_vel: linear.x = %f, angular.z = %f", cmd_vel.linear.x, cmd_vel.angular.z);
  }

  // 📌 Función para cambiar de estado
  void BumpGoBehavior::go_state(int new_state)
  {
    state_ = new_state;  // 📌 Guarda el nuevo estado
    state_ts_ = now();  // 📌 Actualiza la marca de tiempo del cambio de estado
  }

  // 📌 Comprueba si el robot debe pasar de FORWARD a AVOID (si detecta un choque)
  bool BumpGoBehavior::check_forward_2_avoid()
  {
    static bool bumper_was_pressed = false;

    if (last_bump_->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED) {
        if (!bumper_was_pressed) {  
            bumper_was_pressed = true;  // 📌 Evita que se repita el cambio de estado varias veces seguidas
            return true;  // 📌 Si el bumper se presionó por primera vez, cambia a AVOID
        }
    } else {
        bumper_was_pressed = false;  // 📌 Reinicia la detección cuando el bumper se libera
    }
    
    return false;
  }

  // 📌 Comprueba si el robot ha retrocedido suficiente tiempo y debe comenzar a girar
  bool BumpGoBehavior::check_avoid_2_turn()
  {
    return (now() - state_ts_) > AVOID_TIME;  // 📌 Si ha pasado AVOID_TIME, pasa a TURN
  }

  // 📌 Comprueba si el robot ha girado suficiente tiempo y debe volver a avanzar
  bool BumpGoBehavior::check_turn_2_forward()
  {
    return (now() - state_ts_) > TURN_TIME;  // 📌 Si ha pasado TURN_TIME, pasa a FORWARD
  }

}  // namespace bumpgo

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
