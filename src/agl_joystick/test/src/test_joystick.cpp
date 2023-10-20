/*
 *  Copyright 2022 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé, David Valencia
 *  Contact: support.idi@movvo.eu
 *
 */

#pragma GCC diagnostic ignored "-Wpedantic"
#include "test_joystick.hpp"

using ::testing::Return;

TEST_F(TestJoystick, TestInitializeTrue)
{
   //Execute
   bool res = joystick_->Initialize();
   //Assert
   ASSERT_EQ(true, res);
}

TEST_F(TestJoystick, TestResetVelocities)
{
  //Prepare
   geometry_msgs::msg::Twist machine_vel;

   //Assert
   ASSERT_EQ(machine_vel, joystick_->twist_);
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
