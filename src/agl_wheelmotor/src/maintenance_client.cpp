#include "rclcpp/rclcpp.hpp"
#include <ageve_interfaces/srv/maintenance.h>
 
int main(int argc, char **argv)
 {
   rclcpp::init(argc, argv, "maintenance_client");
   rclcpp::NodeHandle nh;
   rclcpp::ServiceClient client = nh.serviceClient<roboteq_motor_controller_driver::Maintenance>("maintenance_service");
   roboteq_motor_controller_driver::Maintenance srv;
   srv.request.user_input = argv[1];

   
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
