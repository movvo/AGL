/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Carles Vela, Martí Bolet
   Contact: support.idi@movvo.eu
*/


#include "agl_imu/imu_factory.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(agl_imu::ImuFactory::create());
    rclcpp::shutdown();

    return 0;
}
