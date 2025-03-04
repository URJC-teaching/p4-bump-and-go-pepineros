// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "rclcpp/rclcpp.hpp"

// Importamos el mensaje "Twist" de ROS para enviar comandos de velocidad                
#include "geometry_msgs/msg/twist.hpp"  

// Importamos las difrenetes bibliotecas de tfs
#include "tf2/LinearMath/Transform.h" 
#include "tf2/transform_datatypes.h"                
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Importa la definición de la clase Controller desde su archivo de cabecera
#include "go_fwd_and_turn/Controller.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace go_fwd_and_turn
{


// Iniciamos el nodo de ROS y el listener con el buffer 
Controller::Controller():Node("Controller"), tf_listener_(tf_buffer_)  
{
  // Publicamos en "cms_vel" para mover el Kobuki
  cmd_publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);  
  
  init();
  // Ejecutamos la funcion control_cycle() cada 200ms
  timer_ = create_wall_timer(200ms, std::bind(&Controller::control_cycle, this));
}


// Inciamos el nodo
void Controller::init(){
  // Hacemos un sleep para que se estabilice el sistema al arrancar el nodo (sino salta error)
  sleep(2.0);  

  // Obtenemos la tf inicial de "odom" a "base_footprint".
  auto odom2bf0_msg = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero); 
  
  // Convertimos el mensaje de tf a un objeto tf2::Transform y lo guardamos en odom2bf0_.
  tf2::fromMsg(odom2bf0_msg, odom2bf0_);  
}


// Función para controlar el movimiento del Kobuki
void Controller::control_cycle(){  
    tf2::Stamped<tf2::Transform> odom2bf1;

    // Obtenemos la tf actual y la guardamos en odom2bf1_msg
    auto odom2bf1_msg = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);  
    tf2::fromMsg(odom2bf1_msg, odom2bf1);  // Pasamos de odom2bf1_msg a odom2bf1

    // Calculamos la diferencia de tfs entre odom2bf0_ y odom2bf1.
    auto bf02bf1 = odom2bf0_.inverse() * odom2bf1;
    /* Usamos inverse para tener la distancia desde base_footprint hasta el odom inicial
    odom2bf0_ --> de odom a base_footprint
    odom2bf0_.inverse() --> de base_footprint a odom */

    double distance;

    // Obtenemos las componentes x e y
    double x = bf02bf1.getOrigin().x();  
    double y = bf02bf1.getOrigin().y();
    
    // Declaramos las variables rol, pitch y angle (para el eje z)
    // double roll, pitch, angle;
    
    // Representamos la rotacion con Quaternion (x, y, z) eje rotacion (w) angulo
    // tf2::Quaternion rotation_quaternion;  // Usamos quaternion porque vimos que es mejor para representar rotaciones

    // Según el estado del robot, decidimos qué acción tomar (maquina de estados)
    switch(state_){
      
      // Kobuki en estado FORWARD
      case FORWARD:  
        // Calculamos la distancia utilizando Pitágoras y seguimos avanzando       
        distance = sqrt(x*x + y*y);  
        message_.linear.x = 0.3;

        // Si la distancia es 1 metro, cambiamos de estado
        if(check_forward_2_stop(distance)){
          go_state(STOP);
        }
        break;
      
      // Kobuki en estado STOP
      case STOP:

        // Velocidad lineal y angular a zero
        message_.linear.x = 0.0;  
        message_.angular.z = 0.0;
        
        break;
    }
    // Publicamos el mensaje en el topic "cmd_vel" con las velocidades calculadas
    cmd_publisher_->publish(message_);  
}


// Funcion para cambiar el estado del Kobuki
void Controller::go_state(int new_state){
  // Asignamos el nuevo estado
  state_ = new_state;

}


// Función para verificar el cambio de TURN a STOP
bool Controller::check_forward_2_stop(double distance){
  // Si la distancia es mayor o igual a 1 metro, cambia de estado
  return distance >= 1;
}
}

