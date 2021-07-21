#include "rclcpp/rclcpp.hpp"
#include <ageve_interfaces/srv/command.h>
 
int main(int argc, char **argv)
 {
   rclcpp::init(argc, argv, "command_client");
   rclcpp::NodeHandle nh;
   rclcpp::ServiceClient client = nh.serviceClient<roboteq_motor_controller_driver::Command>("command_service");
   roboteq_motor_controller_driver::Command srv;
   srv.request.user_input = argv[1];
   srv.request.channel = atoll(argv[2]);
   srv.request.value = atoll(argv[3]);
   if (client.call(srv))
   {
     RCLCPP_INFO("success!");
   }
   else
   {
     RCLCPP_ERROR("Failed to call service");
     return 1;
   }
  
   return 0;
 }
