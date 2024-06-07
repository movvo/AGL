/*
 *  Copyright 2022 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé, David Valencia
 *  Contact: support.idi@movvo.eu
 *
 */

#include "agl_joystick/joystick.hpp"

#include "gmock/gmock.h"

using ::testing::_;

// TODO: Mock class for rclcpp::Node
// class Mock_nh: public rclcpp::Node
// {
// public:
//     MOCK_METHOD(const char *, get_name, (), (const));
//     MOCK_METHOD(rclcpp::Logger, get_logger, (), (const));
// };

class TestJoystick: public ::testing::Test {
protected:
    void SetUp() override 
    {   
        // Create node
        nh_  = rclcpp::Node::make_shared("test_node");

        //Expected Set up calls

        //SetUp class
        joystick_ = new agl_joystick::Joystick(nh_);

        //Expected parameters
        nh_->set_parameter(rclcpp::Parameter("cmd_vel_topic", "cmd_vel_joystick"));
        nh_->set_parameter(rclcpp::Parameter("commands.axes_left_pad_left_right", 0));
        nh_->set_parameter(rclcpp::Parameter("commands.axes_left_pad_up_down", 1));
        nh_->set_parameter(rclcpp::Parameter("commands.axes_right_pad_left_right", 2));
        nh_->set_parameter(rclcpp::Parameter("commands.axes_right_pad_up_down", 3));
        nh_->set_parameter(rclcpp::Parameter("commands.axes_r2", 4));
        nh_->set_parameter(rclcpp::Parameter("commands.axes_l2", 5));
        nh_->set_parameter(rclcpp::Parameter("commands.axes_arrow_left_right", 6));
        nh_->set_parameter(rclcpp::Parameter("commands.axes_arrow_up_down", 7));
        nh_->set_parameter(rclcpp::Parameter("commands.button_a", 0));
        nh_->set_parameter(rclcpp::Parameter("commands.button_b", 1));
        nh_->set_parameter(rclcpp::Parameter("commands.button_x", 3));
        nh_->set_parameter(rclcpp::Parameter("commands.button_y", 4));
        nh_->set_parameter(rclcpp::Parameter("commands.button_l1", 6));
        nh_->set_parameter(rclcpp::Parameter("commands.button_r1", 7));
        nh_->set_parameter(rclcpp::Parameter("commands.button_l2", 8));
        nh_->set_parameter(rclcpp::Parameter("commands.button_select", 10));
        nh_->set_parameter(rclcpp::Parameter("commands.button_start", 11));
        nh_->set_parameter(rclcpp::Parameter("commands.button_pad_left", 13));
        nh_->set_parameter(rclcpp::Parameter("commands.button_pad_right", 14));
        nh_->set_parameter(rclcpp::Parameter("speed.linear_gain", 1.0));
    };

    void TearDown() override
    {
        delete joystick_;
    };

  std::shared_ptr<rclcpp::Node> nh_;
  agl_joystick::Joystick * joystick_;
};