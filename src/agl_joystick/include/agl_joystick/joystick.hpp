/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Iñaki Lorente
   Contact: support.idi@ageve.net
*/

#ifndef AGL_JOYSTICK_JOYSTICK_HPP_
#define AGL_JOYSTICK_JOYSTICK_HPP_

#include <chrono>
#include <memory>
#include <string>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "geometry_msgs/msg/twist.hpp"

// AGEVE
#include "ageve_interfaces/msg/diagnostics.hpp"
#include "ageve_utils/sm/StateMachine.hpp"   

// OWN node
#include "errors.hpp"

// STATES
#define  UNCONFIGURED   0
#define  STANDBY        1
#define  RUN            2
#define  SHUTDOWN       3
#define  FAULT          4

//================================================
class Joystick : public rclcpp::Node, public ageve_utils::sm::StateMachine
//================================================
{
 //---------------------------------- 
 public:
 //----------------------------------
  explicit Joystick();

  // FUNCIONES ESTADO DEL NODO
  void Unconfigured();
  void StandBy();
  void Run();
  void Fault();
  void ShutDown();

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr  msg);

  typedef struct
  {
    rclcpp::Parameter  autorun;
    rclcpp::Parameter  axes_pad_left_right;
    rclcpp::Parameter  axes_pad_up_down;
    rclcpp::Parameter  axes_r2;
    rclcpp::Parameter  axes_l2;
    rclcpp::Parameter  axes_arrow_left_right;
    rclcpp::Parameter  axes_arrow_up_down;
    rclcpp::Parameter  button_a;
    rclcpp::Parameter  button_b;
    rclcpp::Parameter  button_x;
    rclcpp::Parameter  button_y;
    rclcpp::Parameter  button_l1;
    rclcpp::Parameter  button_r1;
    rclcpp::Parameter  button_select;
    rclcpp::Parameter  button_start;
    rclcpp::Parameter  button_pad_left;
    rclcpp::Parameter  button_pad_right;
    rclcpp::Parameter  linear_gain;
    rclcpp::Parameter  angular_gain;
    rclcpp::Parameter  velocity;
  } configuration;

  configuration parameters;

 //----------------------------------
 private:
 //----------------------------------
  /*==================== 
    Publishers,
    Subscribers and
    Timers
  ====================*/
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  /*==================== 
    Variables
  ====================*/
  double velocity = 1.0;
  bool joy_analog = false;

  /*==================== 
    Functions
  ====================*/
  bool Initialize();


};
#endif // AGL_JOYSTICK_JOYSTICK_HPP_