/*
 *  Copyright 2022 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé, David Valencia
 *  Contact: support.idi@movvo.eu
 *
 */

#include "agl_joystick/joystick.hpp"
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace agl_joystick {

Joystick::Joystick(rclcpp::Node::SharedPtr nh) : 
    nh_(nh)
{
    // Declare parameters
    nh_->declare_parameter("cmd_vel_topic", "cmd_vel");
    nh_->declare_parameter("commands.axes_left_pad_left_right", 0);
    nh_->declare_parameter("commands.axes_left_pad_up_down", 1);
    nh_->declare_parameter("commands.axes_right_pad_left_right", 2);
    nh_->declare_parameter("commands.axes_right_pad_up_down", 3);
    nh_->declare_parameter("commands.axes_r2", 4);
    nh_->declare_parameter("commands.axes_l2", 5);
    nh_->declare_parameter("commands.axes_arrow_left_right", 6);
    nh_->declare_parameter("commands.axes_arrow_up_down", 7);
    nh_->declare_parameter("commands.button_a", 0);
    nh_->declare_parameter("commands.button_b", 1);
    nh_->declare_parameter("commands.button_x", 3);
    nh_->declare_parameter("commands.button_y", 4);
    nh_->declare_parameter("commands.button_l1", 6);
    nh_->declare_parameter("commands.button_r1", 7);
    nh_->declare_parameter("commands.button_l2", 8);
    nh_->declare_parameter("commands.button_select", 10);
    nh_->declare_parameter("commands.button_start", 11);
    nh_->declare_parameter("commands.button_pad_left", 13);
    nh_->declare_parameter("commands.button_pad_right", 14);
    nh_->declare_parameter("speed.linear_gain", 1.0);
    nh_->declare_parameter("speed.max_linear_vel", 0.2);
    nh_->declare_parameter("speed.angular_gain", 1.0);
    nh_->declare_parameter("speed.max_angular_vel", 0.2);
}

//================================================
Joystick::~Joystick()
//================================================
{
    
}

bool Joystick::Initialize()
{
    // Get parameters
    try
    {
        parameters_.cmd_vel_topic = nh_->get_parameter("cmd_vel_topic"); 
        parameters_.axes_left_pad_left_right = nh_->get_parameter("commands.axes_left_pad_left_right"); 
        parameters_.axes_left_pad_up_down = nh_->get_parameter("commands.axes_left_pad_up_down");
        parameters_.axes_right_pad_left_right = nh_->get_parameter("commands.axes_right_pad_left_right"); 
        parameters_.axes_right_pad_up_down = nh_->get_parameter("commands.axes_right_pad_up_down"); 
        parameters_.axes_r2 = nh_->get_parameter("commands.axes_r2"); 
        parameters_.axes_l2 = nh_->get_parameter("commands.axes_l2"); 
        parameters_.axes_arrow_left_right = nh_->get_parameter("commands.axes_arrow_left_right"); 
        parameters_.axes_arrow_up_down = nh_->get_parameter("commands.axes_arrow_up_down"); 
        parameters_.button_a = nh_->get_parameter("commands.button_a"); 
        parameters_.button_b = nh_->get_parameter("commands.button_b"); 
        parameters_.button_x = nh_->get_parameter("commands.button_x"); 
        parameters_.button_y = nh_->get_parameter("commands.button_y"); 
        parameters_.button_l1 = nh_->get_parameter("commands.button_l1"); 
        parameters_.button_r1 = nh_->get_parameter("commands.button_r1");
        parameters_.button_l2 = nh_->get_parameter("commands.button_l2"); 
        parameters_.button_select = nh_->get_parameter("commands.button_select");
        parameters_.button_start = nh_->get_parameter("commands.button_start"); 
        parameters_.button_pad_left = nh_->get_parameter("commands.button_pad_left"); 
        parameters_.button_pad_right = nh_->get_parameter("commands.button_pad_right");
        parameters_.linear_gain = nh_->get_parameter("speed.linear_gain"); 
        parameters_.max_linear_vel = nh_->get_parameter("speed.max_linear_vel");
        parameters_.angular_gain = nh_->get_parameter("speed.angular_gain"); 
        parameters_.max_angular_vel = nh_->get_parameter("speed.max_angular_vel");
    }
    catch (std::runtime_error &exc)
    {
        RCLCPP_ERROR(nh_->get_logger(), exc.what());
        return false;
    }

    // Initialize publishers
    std::string cmd_vel_topic  = "/"+parameters_.cmd_vel_topic.as_string();
	cmd_vel_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(rclcpp::KeepLast(10)));

    // Initialize subscribers
    joy_sub_ = nh_->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(rclcpp::KeepLast(10)),
                                                                    std::bind(&Joystick::JoyCallback, this, _1));
    
    joystick_timer_ = nh_->create_wall_timer(10ms, std::bind(&Joystick::RunJoystick,this));
    
    ResetVelocities();
    first_connection_ = true;
    counter_ = 0;
    RCLCPP_INFO(nh_->get_logger(), "Max linear vel setted to: %f",parameters_.max_linear_vel.as_double());
    RCLCPP_INFO(nh_->get_logger(), "Max angular vel setted to: %f",parameters_.max_angular_vel.as_double());
    RCLCPP_INFO(nh_->get_logger(), "Joystick succesfully initialized");

    return true;
}

void Joystick::ResetVelocities()
{
    twist_.linear.x = 0.0;
    twist_.linear.y = 0.0;
    twist_.linear.z = 0.0;
    twist_.angular.x = 0.0;
    twist_.angular.y = 0.0;
    twist_.angular.z = 0.0;
}

void Joystick::RunJoystick() 
{
    if (JoyExists("/dev/input/js0")) {
        if (!joy_connected_) {
            counter_ = 0;
        }
        counter_++;
        on_error_ = false;
        joy_connected_ = true;
        PublishCmdVel();
        if (counter_ == 1) {
            RCLCPP_INFO(nh_->get_logger(), "Joystick Connected");
        }
    }
    else {
        if (joy_connected_) {
            counter_ = 0;
        }
        counter_++;
        on_error_ = true;
        first_connection_ = true;
        ResetVelocities();
        joy_connected_ = false;
        if (counter_ == 1) {
            RCLCPP_WARN(nh_->get_logger(), "Joystick NOT Connected");
        }
    }
}

void Joystick::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr  msg)
{
    if (!on_error_) {
        
        if (first_connection_) {
            velocity_ = 0.0;
            angle_ = 0.0;
            if (msg->axes[parameters_.axes_left_pad_up_down.as_int()] == 0 && msg->axes[parameters_.axes_left_pad_left_right.as_int()] == 0
                    && msg->axes[parameters_.axes_right_pad_up_down.as_int()] == 0 && msg->axes[parameters_.axes_right_pad_left_right.as_int()] == 0) {
                first_connection_ = false;
            }
        }
        else {
            velocity_ = parameters_.max_linear_vel.as_double();
            angle_ = parameters_.max_angular_vel.as_double();
            //std::cout << "No entro nunca a esta rutina" << std::endl;
        }
        
        // MOVEMENT LOGIC

        // linear velocity:
        if (msg->axes[parameters_.axes_left_pad_up_down.as_int()] && msg->buttons[parameters_.button_r1.as_int()]) {
            twist_.linear.x = parameters_.linear_gain.as_double()*velocity_*msg->axes[parameters_.axes_left_pad_up_down.as_int()];
        }
        else {
            twist_.linear.x = 0;
        }

        // angular velocity
        if (msg->axes[parameters_.axes_right_pad_left_right.as_int()] && msg->buttons[parameters_.button_r1.as_int()]) {
            twist_.angular.z = parameters_.angular_gain.as_double()*angle_*msg->axes[parameters_.axes_right_pad_left_right.as_int()];
        }      
        else {
            twist_.angular.z = 0;
        }
    }
    joy_timeout_ = 0;
}

void Joystick::PublishCmdVel()
{
    joy_timeout_ ++;

    if (joy_timeout_ > 50) {
        ResetVelocities();

        cmd_vel_pub_->publish(twist_);
    }
    else {
        twist_.linear.y = 0.0;
        twist_.angular.x = 0.0;
        twist_.angular.y = 0.0;
        cmd_vel_pub_->publish(twist_);
    }
}

bool Joystick::JoyExists(const std::string& filename)
{
    std::experimental::filesystem::path FullPath(filename);
    bool joy_exists = std::experimental::filesystem::exists(FullPath);
    return joy_exists;
}

} //namespace agl_joystick