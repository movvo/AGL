/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Carles Vela, Martí Bolet
   Contact: support.idi@movvo.eu
*/
#ifndef WIT_IMU_HPP_
#define WIT_IMU_HPP_

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "tf2/LinearMath/Quaternion.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"

#include "atlas_imu/imu_drivers/driver_abstract.hpp"
#include "atlas_imu/imu_errors.hpp"
// #include "atlas_utils/sm/StateMachine.hpp"
#include "atlas_utils/general/system_funcs.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using namespace LibSerial;

namespace atlas_imu {
namespace imu_drivers {

//================================================
class WitImu : public DriverAbstract
//================================================
{
  public:
  	// Constructor
	WitImu(std::string driver_name);
	~WitImu();

	// Method to configure parameters
    void Initialize(); 

	bool Configure();

	// Method to open the connection
    bool Connect();

	// Publish IMU info through ROS2 topic
    sensor_msgs::msg::Imu GetImuMsg();

	// Get IMU orientation(angles)
    std::vector<float> getAngles();

    // Get IMU angular velocity
    std::vector<float> getAngVel();

    // Get IMU linear accelerations
    std::vector<float> getAngAcc();

	// Try to solve errors if posible
	bool ErrorHandle();

  private:
    // Parameter
    std::string frame_id_;
    std::string serial_port_;
    int baudrate_;

    /*****************************
     *  FUNCIONES PROPIAS DEL NODO
    ******************************/
   // Open serial port to communicate with IMU
    bool openSerial(std::string port, int baudrate);

	// Close serial port to communicate with IMU
	void closeSerial();

	// Check if exists the serial port specified
	bool checkIfSerial(std::string serial);

	// Util function to transform from int to LibSerial::BaudRate
	LibSerial::BaudRate baudrateOf(int baudrate);

	uint16_t ModRTU_CRC(std::vector<uint8_t> buf);
	int readBaudRate();
	std::vector<double> readAccelerationBias();
	std::vector<float> readAngularVelocityBias();
	std::vector<int> writeQuery(std::vector<uint8_t> query);
	void writeCommand(std::vector<uint8_t> command, bool read_response=true);

	/*****************************
	 *  VARIABLES PRIVADAS
	******************************/

	// Publisher
	// rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

	// Service
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr calibration_srv_;
	void calibrateAcceleration(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);

	
    
	/*==================== 
		Others
	====================*/

	// Serial comm.
	bool serial_is_opened_ = false;
	SerialPort serial_;

	// Covariancie
	std::vector<float> imu_acceleration_bias_;
	std::vector<float> imu_angular_velocity_bias_;
};

} // namespace imu_drivers
} // namespace atlas_imu

#endif // WIT_IMU_HPP_