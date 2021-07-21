/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Iñaki Lorente
   Contact: support.idi@ageve.net
*/

#include "agl_wheelmotor/wheelmotor.hpp"


using std::placeholders::_1;

Wheelmotor::Wheelmotor() :
     Node("Wheelmotor")
{
  RCLCPP_INFO(this->get_logger(), "Wheel motors ON");

  nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  // Subscriber to cmd_vel, every new cmd_vel the cmd_vel_callback
  // will send the command to the motors
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
      qos, 
      std::bind(&Wheelmotor::cmd_vel_callback, this, _1));

  add_motors();
  add_odom();
}

void Wheelmotor::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");
 
  this->declare_parameter("motor.usb_port");
  this->declare_parameter("motor.id_right");
  this->declare_parameter("motor.id_left");

  this->get_parameter_or<std::string>("motor.dev", config_.usb_port, "/dev/ttyUSB0");
  this->get_parameter_or<uint8_t>("motor.id_right", config_.id_right, 1);
  this->get_parameter_or<uint8_t>("motor.id_left", config_.id_left, 2);


  // Serial comm.
  // using LibSerial::SerialPort;
  // using LibSerial::SerialStream;
  // using LibSerial::BaudRate;

  // SerialPort serial_port;
  // SerialStream serial_stream;

  // // Open hardware serial ports using the Open() method.
  // serial_port.Open(config_.usb_port);
  // serial_stream.Open(config_.usb_port);
  // // Set the baud rates.
  // serial_port.SetBaudRate(BaudRate::BAUD_115200);
  // serial_stream.SetBaudRate(BaudRate::BAUD_115200);

}

void Wheelmotor::add_odom()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels odometry and joint states");
  wheel_odom_ = std::make_unique<Odometry>(nh_);

}

void Wheelmotor::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  //int32_t sp_vel_linear = static_cast<int32_t>(msg->linear.x);
  //int32_t sp_vel_angular = static_cast<int32_t>(msg->angular.z);


  std::cout << "Good";
          
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Wheelmotor>());
  rclcpp::shutdown();

  return 0;
}