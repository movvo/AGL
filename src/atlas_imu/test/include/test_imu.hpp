/*
   Copyright 2022 @ AGEVE
   ---------------------------------------------------------
   Authors: María Mercadé
   Contact: support.idi@ageve.net
*/

#include "atlas_imu/imu.hpp"

#include "gmock/gmock.h"

using ::testing::_;

class MockSM: public atlas_utils::sm::StateMachineInterface
{
public:
   MOCK_METHOD(void, updateFrame_id, (std::string value), (override));
   MOCK_METHOD(void, updateErrorCritical, (uint32_t value, std::string description), (override));
   MOCK_METHOD(void, updateErrorWarning, (uint32_t value, std::string description), (override));
   MOCK_METHOD(void, updateErrorTimeout, (uint32_t value, std::string description), (override));
   MOCK_METHOD(void, updateErrorDescription, (std::string value), (override));
   MOCK_METHOD(void, updateSolvedCritical, (uint32_t value, std::string description), (override));
   MOCK_METHOD(void, updateSolvedWarning, (uint32_t value, std::string description), (override));
   MOCK_METHOD(void, updateSolvedTimeout, (uint32_t value, std::string description), (override));
   MOCK_METHOD(void, updateSolvedDescription, (std::string value), (override));
   MOCK_METHOD(void, updateState, (uint8_t value), (override));

   /*---------------------------------
      SETTERS / GETTERS
   ---------------------------------*/   
   MOCK_METHOD(void, SetUnconfiguredTimer, (rclcpp::TimerBase::SharedPtr timer), (override));
   MOCK_METHOD(void, SetStandByTimer, (rclcpp::TimerBase::SharedPtr timer), (override));
   MOCK_METHOD(void, SetFaultTimer, (rclcpp::TimerBase::SharedPtr timer), (override));
   MOCK_METHOD(void, SetRunTimer, (rclcpp::TimerBase::SharedPtr timer), (override));
   MOCK_METHOD(void, SetShutdownTimer, (rclcpp::TimerBase::SharedPtr timer), (override));

   MOCK_METHOD(rclcpp::TimerBase::SharedPtr, GetUnconfiguredTimer, (), (const, override));
   MOCK_METHOD(rclcpp::TimerBase::SharedPtr, GetStandByTimer, (), (const, override));
   MOCK_METHOD(rclcpp::TimerBase::SharedPtr, GetFaultTimer, (), (const, override));
   MOCK_METHOD(rclcpp::TimerBase::SharedPtr, GetRunTimer, (), (const, override));
   MOCK_METHOD(rclcpp::TimerBase::SharedPtr, GetShutdownTimer, (), (const, override));

   /*---------------------------------
      CANCELS / CHECKERS
   ---------------------------------*/
   MOCK_METHOD(void, CancelUnconfiguredTimer, (), (override));
   MOCK_METHOD(void, CancelStandByTimer, (), (override));
   MOCK_METHOD(void, CancelFaultTimer, (), (override));
   MOCK_METHOD(void, CancelRunTimer, (), (override));
   MOCK_METHOD(void, CancelShutdownTimer, (), (override));

   MOCK_METHOD(bool, IsOnFault, (), (const, override));
   MOCK_METHOD(bool, IsOnRun, (), (const, override));
   MOCK_METHOD(bool, IsCritical, (uint32_t errorBit), (const, override));
   
   /*---------------------------------
         FUNCIONES CAMBIO DE ESTADO
   ---------------------------------*/
   MOCK_METHOD(bool, ToggleState, (rclcpp::TimerBase::SharedPtr timer), (override));
   MOCK_METHOD(void, ToggleState_srv_callback, (
      const std::shared_ptr<atlas_interfaces::srv::ToggleState::Request> request,
      std::shared_ptr<atlas_interfaces::srv::ToggleState::Response> response), (override));
   MOCK_METHOD(void, Request_Toggle, (std::string node_name, uint8_t state2go), (override));

};

class WrapImuNode: public atlas_imu::ImuNode
{
public:
   WrapImuNode(std::shared_ptr<atlas_utils::sm::StateMachineInterface> /*sm*/,
                 rclcpp::Node::SharedPtr /*nh*/) : atlas_imu::ImuNode() {}
   ~WrapImuNode() {};
};

class TestImuNode: public ::testing::Test {
protected:
   void SetUp() override 
   {   
      // Create node
      nh_  = rclcpp::Node::make_shared("test_node");
      mock_sm_ = std::make_shared<MockSM>();
      // SetUp class
      io_ = std::make_unique<WrapImuNode>(mock_sm_, nh_);
   }
   void TearDown() override
   {
      io_.reset();
   };

   std::shared_ptr<MockSM> mock_sm_;
   std::shared_ptr<rclcpp::Node> nh_;
   std::unique_ptr<WrapImuNode> io_;
};