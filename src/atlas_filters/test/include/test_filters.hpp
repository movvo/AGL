/*
   Copyright 2022 @ AGEVE
   ---------------------------------------------------------
   Authors: María Mercadé
   Contact: support.idi@ageve.net
*/

#include "atlas_filters/filter_node.hpp"

#include "gmock/gmock.h"

using ::testing::_;

class WrapFilterNode: public atlas_filters::FilterNode
{
public:
   WrapFilterNode(rclcpp::Node::SharedPtr /*nh*/) : atlas_filters::FilterNode() {}
   ~WrapFilterNode() {};
};

class TestFilterNode: public ::testing::Test {
protected:
   void SetUp() override 
   {   
      rclcpp::init(0, nullptr);
      // Create node
      nh_  = rclcpp::Node::make_shared("test_node");
      // SetUp class
      filter_ = std::make_unique<WrapFilterNode>(nh_);
   }
   void TearDown() override
   {
      rclcpp::shutdown();
      filter_.reset();
   };

   std::shared_ptr<rclcpp::Node> nh_;
   std::unique_ptr<WrapFilterNode> filter_;
};