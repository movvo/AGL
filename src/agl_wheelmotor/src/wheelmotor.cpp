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
    "Control/cmd_vel",
      qos, 
      std::bind(&Wheelmotor::cmd_vel_callback, this, _1));

  add_motors();
  add_odom();
}

void Wheelmotor::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");
 
  this->declare_parameter("motor.usb_port");
  this->declare_parameter("motor.max_vel");
  this->declare_parameter("motor.min_vel");

  this->get_parameter_or<std::string>("motor.dev", config_.usb_port, "/dev/ttyUSB0");
  this->get_parameter_or<uint8_t>("motor.max_vel", config_.max_vel, 50);
  this->get_parameter_or<uint8_t>("motor.min_vel", config_.min_vel, 0);

  // Open hardware serial ports using the Open() method.
  serial_port.Open(config_.usb_port);
  serial_stream.Open(config_.usb_port);
  // Set the baud rates.
  serial_port.SetBaudRate(BaudRate::BAUD_115200);
  serial_stream.SetBaudRate(BaudRate::BAUD_115200);

}

void Wheelmotor::add_odom()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels odometry and joint states");
  wheel_odom_ = std::make_unique<Odometry>(nh_);

}

void Wheelmotor::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{

  std::cout << "--------------- CMD_VEL RECEIVED ------------- " << std::endl;

  // Conversion linear/angular a PMW
  if (((msg->linear.x >= config_.min_vel) and (msg->linear.x<=config_.max_vel)) &
     ((msg->angular.z >= config_.min_vel) and (msg->angular.z<=config_.max_vel))) 
  {
    // Set velocidades
    sp_vel_right = msg->linear.x + msg->angular.z;
    sp_vel_left = msg->linear.x - msg->angular.z;

    // Set porcentaje velocidad
    pmw_right = (sp_vel_right * 100)/config_.max_vel;
    pmw_left = (sp_vel_left * 100)/config_.max_vel;


    // Envio por puerto serie a Arduino
    serial_port.WriteByte((char) pmw_left);
    serial_port.WriteByte((char) pmw_right);
    serial_stream << pmw_left;
    serial_stream << pmw_right;

    std::cout << "--------------- PMW_LEFT: " << pmw_left << std::endl;
    std::cout << "--------------- PMW_RIGHT: " << pmw_right << std::endl;


  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(),"SET VELOCIDAD INCORRECTO! - VELOCIDAD SUPERA RANGO MAX O INFERIOR A MIN");
  }
          
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