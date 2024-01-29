/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Carles Vela, Martí Bolet
   Contact: support.idi@movvo.eu
*/


#include "atlas_imu/imu_factory.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(atlas_imu::ImuFactory::create());
    rclcpp::shutdown();

    return 0;
}
