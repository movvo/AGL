/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Martí Bolet, Albert Arlà
   Contact: support.idi@ageve.net
*/

#ifndef CONTROL_MANAGER_NODE_HPP
#define CONTROL_MANAGER_NODE_HPP

//C++ Standard 
#include <string>
#include <iostream> 
#include <vector>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp" // Dynamic reconf


//Own interfaces
#include "ageve_utils/sm/StateMachine.hpp"
#include "ageve_utils/general/system_funcs.hpp"
#include "ageve_interfaces/msg/control.hpp"
#include "ageve_interfaces/msg/laser_driver_header.hpp"
#include "ageve_interfaces/msg/digital_inputs.hpp"
#include "ageve_interfaces/msg/digital_outputs.hpp"

// Utils
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>

// DEFINED VARIABLES
typedef enum {
    MANUAL_NAVIGATION     = ageve_interfaces::msg::Control::MANUAL,
    AUTONOMOUS_NAVIGATION = ageve_interfaces::msg::Control::AUTONOMOUS,
    PAUSE_NAVIGATION      = ageve_interfaces::msg::Control::PAUSE
} navigation_mode_t;

using namespace std::chrono_literals;


//================================================
class ControlManager : public rclcpp::Node, public ageve_utils::sm::StateMachine
//================================================
{
    //----------------------------------
    public:
    //----------------------------------
        ControlManager();

        // FUNCIONES ESTADO DEL NODO
        void Unconfigured();
        void StandBy();
        void Run();
        void Fault();
        void ShutDown();
       
    //----------------------------------
    private:
    //----------------------------------
        // Modo de navegación
        navigation_mode_t navigation_mode;

        // Subsripciones a los topicos de velocidad
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr autonomous_sub;
        // Callbacks de las subscripciones de los topicos de velocidad
        void manual_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void autonomous_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        // Publicador de velocidad final
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

        // Publicador para informar de estados
        ageve_interfaces::msg::Control control_msg_;
        rclcpp::Publisher<ageve_interfaces::msg::Control>::SharedPtr control_publisher;
        void controlControl();

        // Esta función se llamará cuando haya un cambio de valor en el selector de Modo de Navegación
        void switchMode(navigation_mode_t mode);
        // Esta función sirve para comprobar si hay un joystick conectado.
        bool checkIfJoystick();

        // Laser info subscriptions
        rclcpp::Subscription<ageve_interfaces::msg::LaserDriverHeader>::SharedPtr fornt_laser_info;
        rclcpp::Subscription<ageve_interfaces::msg::LaserDriverHeader>::SharedPtr back_laser_info;
        void LaserInfoCallback(const ageve_interfaces::msg::LaserDriverHeader::SharedPtr msg);

        // ADAM Module Subs and Pub
        rclcpp::Publisher<ageve_interfaces::msg::DigitalOutputs>::SharedPtr dig_out_pub;
        ageve_interfaces::msg::DigitalInputs dig_out_msg;

        rclcpp::Subscription<ageve_interfaces::msg::DigitalInputs>::SharedPtr dig_in_subs;
        void ADAM_Inputs_Callback(const ageve_interfaces::msg::DigitalInputs::SharedPtr msg);


        geometry_msgs::msg::Twist::SharedPtr AdjustVelocity(geometry_msgs::msg::Twist::SharedPtr msg);

        // Varibales warning zone laser
        bool warn_zone1_front = false;
        bool warn_zone2_front = false;
        bool protected_zone_front = false;
        bool warn_zone1_back = false;
        bool warn_zone2_back = false;
        bool protected_zone_back = false;
        uint16_t active_zone_set_laser_front;
        uint16_t active_zone_set_laser_back;

    /*****************************
     *  RECONFIGURACIÓN DINÁMICA
    ******************************/
    rcl_interfaces::msg::SetParametersResult dyn_reconf_callback(const std::vector<rclcpp::Parameter> & parameters);

    // PARAMETERS
    typedef struct {
        rclcpp::Parameter autorun;
        rclcpp::Parameter manual_pkg;
        rclcpp::Parameter autonomous_pkg;
        rclcpp::Parameter initial_mode;
        rclcpp::Parameter scan_front;
        rclcpp::Parameter scan_back;
    } configuration_t;

    configuration_t parameters;

    // FUNCTIONS NODE
    bool Initialize();

};

#endif //CONTROL_MANAGER_NODE_HPP