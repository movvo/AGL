/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Albert Arlà Romero
   Contact: support.idi@ageve.net
*/

#include "atlas_filters/filter_node.hpp"

int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<atlas_filters::FilterNode>());
    rclcpp::shutdown();
    return 0;
  }