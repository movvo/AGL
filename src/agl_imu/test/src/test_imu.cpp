/*
   Copyright 2022 @ AGEVE
   ---------------------------------------------------------
   Authors: María Mercadé
   Contact: support.idi@ageve.net
*/
#pragma GCC diagnostic ignored "-Wpedantic"
#include "test_imu.hpp"

using ::testing::Return;

TEST_F(TestImuNode, TestThisPackagesPlease)
{

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
