/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Marti Bolet
   Contact: support.idi@ageve.net
*/

#include "rclcpp/rclcpp.hpp"
#include "me00_control/control_manager.hpp"

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControlManager>());
	rclcpp::shutdown();
	return 0;
}