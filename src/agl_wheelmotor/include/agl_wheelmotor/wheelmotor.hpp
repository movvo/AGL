/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Iñaki Lorente
   Contact: support.idi@ageve.net
*/

#ifndef AGL_WHEELMOTOR_WHEELMOTOR_HPP_
#define AGL_WHEELMOTOR_WHEELMOTOR_HPP_

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "agl_wheelmotor/diff_odom.hpp"


class Wheelmotor : public rclcpp::Node
{
 public:
  explicit Wheelmotor();

  typedef struct
  {
    std::string usb_port;
    uint8_t id_right;
    uint8_t id_left;
    uint32_t pwm;
  } Motors;

  Motors config_;

 private:
  /*==================== 
    Publishers,
    Subscribers and
    Timers
  ====================*/
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

 /*==================== 
    Functions
  ====================*/
  void add_motors();
  void add_odom();
  void update_odom();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /*==================== 
    Others
  ====================*/
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> wheel_odom_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

};
#endif // AGL_WHEELMOTOR_HPP_