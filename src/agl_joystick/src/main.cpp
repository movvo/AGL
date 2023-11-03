/*
 *  Copyright 2022 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé, David Valencia
 *  Contact: support.idi@movvo.eu
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "agl_joystick/joystick.hpp"

int main(int argc, char * argv[]) {
    //init rclcpp
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    //Init dependencies
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("node_name");
    auto joystick = std::make_shared<agl_joystick::Joystick>(nh);
    if (!joystick->Initialize()) {
        std::cerr << "ERROR INITIALIZING JOYSTICK" << std::endl;
    }

    //Spin
    executor.add_node(nh);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}