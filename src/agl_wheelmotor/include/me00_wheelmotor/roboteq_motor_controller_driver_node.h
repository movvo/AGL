#ifndef ROBOTEQ_MOTOR_CONTROLLER_DRIVER_MAIN_H
#define ROBOTEQ_MOTOR_CONTROLLER_DRIVER_MAIN_H

// ROS2
#include "rclcpp/rclcpp.hpp"

// ROS2 standard msgs
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

// Custom messages/services
#include "ageve_interfaces/srv/config.hpp"
#include "ageve_interfaces/srv/command.hpp"
#include "ageve_interfaces/srv/maintenance.hpp"
#include "ageve_interfaces/msg/channel_values.hpp"
#include "ageve_interfaces/msg/diagnostics.hpp"

// Utils
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>

// ODVA EtherNet/IP
#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"

// Other classes
#include "me00_wheelmotor/diff_odom.h"
#include "me00_wheelmotor/querylist.h"
#include "me00_wheelmotor/controller.h"
#include "me00_wheelmotor/controller_eip.h"
#include "me00_wheelmotor/anybus_constants.h"
#include "me00_wheelmotor/roboteq_constants.h"
#include "me00_wheelmotor/errors.h"

namespace roboteq {

class Driver_Node : public rclcpp::Node {
  public:
	// Constructor
	Driver_Node();
	virtual ~Driver_Node();

	// Odometry Class
	std::unique_ptr<Odometry> odom_;
	bool use_odom;

	// Controller Class
	std::unique_ptr<Controller> controller_;

	// Controller Class for EtherNet/IP Protocol
	rclcpp::TimerBase::SharedPtr connect_timmer_;
	std::unique_ptr<roboteq::Controller_EIP> controller_eip_;
	void connect_callback();

	// Subscriptors (and callback) and publishers
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub; 
	void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr read_publisher;
	
	std::vector<rclcpp::Publisher<ageve_interfaces::msg::ChannelValues>::SharedPtr> publisherVecH;
	std::vector<rclcpp::Publisher<ageve_interfaces::msg::ChannelValues>::SharedPtr> publisherVecL;
	std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publisherVecG;

	// ROS2 Services and callbacks functions
	rclcpp::Service<ageve_interfaces::srv::Config>::SharedPtr configsrv;
	rclcpp::Service<ageve_interfaces::srv::Command>::SharedPtr commandsrv;
	rclcpp::Service<ageve_interfaces::srv::Maintenance>::SharedPtr maintenancesrv;	

	void configservice(const std::shared_ptr<ageve_interfaces::srv::Config::Request> req, std::shared_ptr<ageve_interfaces::srv::Config::Response> res);
	void commandservice(const std::shared_ptr<ageve_interfaces::srv::Command::Request> req,	std::shared_ptr<ageve_interfaces::srv::Command::Response> res);
	void maintenanceservice(const std::shared_ptr<ageve_interfaces::srv::Maintenance::Request> req, std::shared_ptr<ageve_interfaces::srv::Maintenance::Response> res);
	
	// Diagnostics variables
	ageve_interfaces::msg::Diagnostics diagnostics_msg;
	rclcpp::Publisher<ageve_interfaces::msg::Diagnostics>::SharedPtr diagnostics_publisher;
	rclcpp::TimerBase::SharedPtr timer_;
	void diagnostics_callback();
	typedef struct {
		uint8_t state;
		uint16_t anybus_error;
		uint8_t controller_error;
		uint8_t motor_right_error;
		uint8_t motor_left_error;
	} InternalState;
	InternalState internal_state;

	// Main functions
	bool Initialize();
	void Run();
	
	// Parameters
	int frequencyH;
	int frequencyL;
	int frequencyG;
	
  private:
	std::shared_ptr<rclcpp::Node> nh_;

	int channel;
	
	// Function to transform from an 8bit flag to list of Errors
	// Usage: errors = this->to_fault({error_flag}, this->FF_TO_STR);
	std::vector<std::string> to_fault(int num, std::map<int, std::string> map){
		std::vector<std::string> errors;
		for(int i=0; i<8; i++){
			if(CHECK_BIT(num, i)) {
				errors.push_back(map[CHECK_BIT(num, i)]);
			}
		}
		return errors;
	}

	// Fault Flags to String errors
	std::map<int, std::string> FF_TO_STR {
		{OVERHEAT,      "Overheat Fault"},
		{OVERVOLTAGE,   "Overvoltatge Fault"},
		{UDERVOLTAGE,   "Undervoltatge Fault"},
		{SHORCIRCUIT,   "Short circuit Fault"},
		{EMERCENCYSTOP, "Emergency stop Fault"},
		{SETUPFAIL,     "Motor/Sensor setup Fault"},
		{MOSFETFAIL,    "MOSFET failure Fault"},
		{DEFAULTCONF,   "Default configuration loaded at startup"}
	};

	std::map<int, std::string> FM_TO_STR {
		{AMPSLIMIT,    "Amps Limit currently active"},
		{MOTORSTALLED, "Motor stalled"},
		{LOOPERROR,    "Loop Error detected"},
		{SAFETYSTOP,   "Safety Stop active"},
		{FORWARDLIMIT, "Forward Limit triggered"},
		{REVERSELIMIT, "Reverse Limit triggered"},
		{AMPPSTRIGGER, "Amps Trigger activated"},
	};
}; // Class Driver_Node

}	// namespace
#endif // ROBOTEQ_MOTOR_CONTROLLER_DRIVER_MAIN_H