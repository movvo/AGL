/*
 *  Copyright 2022 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé, David Valencia
 *  Contact: support.idi@movvo.eu
 *
 */

#ifndef JOYSTICK_HPP_
#define JOYSTICK_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <math.h>
#include <experimental/filesystem>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

namespace agl_joystick {
/*!
  @class Joystick
  @brief AGL Joystick Driver Node
*/
class Joystick  {
    public:
        /*!
            @brief Constructor of the Joystick driver class
            @param[in] nh Injected rclcpp node dependency
        */
        Joystick(rclcpp::Node::SharedPtr nh);
        ~Joystick();

        /// @brief It  gets parms and initializes ROS2 publishers and subscribers
        bool Initialize();

        /// @brief Executes joystick driver logic
        void Execute();

        geometry_msgs::msg::Twist twist_; /*!< Machine cmd_vel msg */
        /// @brief Joystick parameters
        typedef struct
        {
            rclcpp::Parameter  cmd_vel_topic;
            rclcpp::Parameter  autorun;
            rclcpp::Parameter  axes_left_pad_left_right;
            rclcpp::Parameter  axes_left_pad_up_down;
            rclcpp::Parameter  axes_right_pad_left_right;
            rclcpp::Parameter  axes_right_pad_up_down;
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
            rclcpp::Parameter  button_l2;
            rclcpp::Parameter  button_select;
            rclcpp::Parameter  button_start;
            rclcpp::Parameter  button_pad_left;
            rclcpp::Parameter  button_pad_right;
            rclcpp::Parameter  linear_gain;
            rclcpp::Parameter  max_linear_vel;
            rclcpp::Parameter  angular_gain;
            rclcpp::Parameter  max_angular_vel;
        } configuration;
        configuration parameters_;

    protected:


    private:
        /*==================== 
            Functions
        ====================*/

        /// @brief 
        void ResetVelocities();
        
        /*!
            @brief It receives commands from the gamepad
            @param[in] msg joy msg
        */
        void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr  msg);

        // /*!
        //   @brief Callback in which the Joystick driver logic is applied
        // */
        void RunJoystick();

        /// @brief Publishes the cmd_vel commands for the machine and the forks
        void PublishCmdVel();

        /// @brief It checks if joystick is connected
        bool JoyExists(const std::string& filename);


        /*==================== 
            Variables
        ====================*/
        bool first_connection_ = true;
        bool on_error_ = true;
        bool joy_connected_ = false;
        double velocity_ = 0.0;
        double angle_ = 0.0;
        uint32_t counter_ = 0;
        uint8_t joy_timeout_ = 0;
        rclcpp::Node::SharedPtr nh_; /*!< RCLCPP Node */
  /*==================== 
  ROS2 Publishers, Subscibers and Timers
  ====================*/
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; /*!< Publisher of the machine cmd_vel */
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_; /*!< Subscription to gamepad commands */
        rclcpp::TimerBase::SharedPtr joystick_timer_; /*!< Timer to keep publishing the joystick cmd_vel messages */
  
};
} //namespace agl_joystick
#endif // JOYSTICK_HPP_