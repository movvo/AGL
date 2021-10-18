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

  this->get_parameter_or<std::string>("motor.dev", config_.usb_port, "/dev/ttyACM0");
  this->get_parameter_or<uint8_t>("motor.max_vel", config_.max_vel, 50);
  this->get_parameter_or<uint8_t>("motor.min_vel", config_.min_vel, 0);

  try
  {
    // Open hardware serial ports using the Open() method.
    serial_port.Open(config_.usb_port);
    RCLCPP_INFO(this->get_logger(), "[OK] Serial port opened...");
    serial_port.SetBaudRate(BaudRate::BAUD_9600);
    RCLCPP_INFO(this->get_logger(), "[OK] Serial port baud rate = 9600");
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;
  }
  catch (const OpenFailed&)
  {
    RCLCPP_INFO(this->get_logger(), "[ERROR] Opening serial port...");
  }
}

void Wheelmotor::add_odom()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels odometry and joint states");
  wheel_odom_ = std::make_unique<Odometry>(nh_);

}

void Wheelmotor::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Conversion linear/angular a PMW
  // --------------------------------

  // LINEAR VEL
  if (static_cast<int>(msg->linear.x) > config_.max_vel){
    mapped_vel_linear = config_.max_vel;
    RCLCPP_INFO(this->get_logger(),"[ERROR] linear velocity out of range, set to max vel");
  }else if (static_cast<int>(msg->linear.x) < config_.min_vel){
    mapped_vel_linear = config_.min_vel;
    RCLCPP_INFO(this->get_logger(),"[ERROR] linear velocity out of range, set to min vel");
  }else{
    mapped_vel_linear = msg->linear.x;
  }

  // ANGULAR VEL
  if (static_cast<int>(msg->angular.z) > config_.max_vel){
    mapped_vel_angular = config_.max_vel;
    RCLCPP_INFO(this->get_logger(),"[ERROR] angular velocity out of range, set to max vel");
  }else if (static_cast<int>(msg->angular.z) < config_.min_vel){
    mapped_vel_angular = config_.min_vel;
    RCLCPP_INFO(this->get_logger(),"[ERROR] angular velocity out of range, set to min vel");
  }else{
    mapped_vel_angular = msg->angular.z;
  }

  // Left and right motor VEL
  sp_vel_right = mapped_vel_linear + mapped_vel_angular;
  sp_vel_left = mapped_vel_linear - mapped_vel_angular;

  // Set porcentaje velocidad
  pmw_right = (sp_vel_right / config_.max_vel)*100;
  pmw_left = (sp_vel_left / config_.max_vel)*100;

  // Send pmw to arduino
  serial_port.WriteByte(static_cast<unsigned char>(pmw_left));
  serial_port.WriteByte(static_cast<unsigned char>(pmw_right));

  std::cout << "\tLeft:\t" << std::to_string(pmw_left) << std::endl
              << "\tRight:\t" << std::to_string(pmw_right) << std::endl
              << std::endl ;
  // }
  // else
  // {
  //   RCLCPP_INFO(this->get_logger(),"SET VELOCIDAD INCORRECTO! - VELOCIDAD SUPERA RANGO MAX O INFERIOR A MIN");
  // }
          
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