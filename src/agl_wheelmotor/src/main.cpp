
#include <me00_wheelmotor/roboteq_motor_controller_driver_node.h>

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<roboteq::Driver_Node>());
	rclcpp::shutdown();
	return 0;
}
