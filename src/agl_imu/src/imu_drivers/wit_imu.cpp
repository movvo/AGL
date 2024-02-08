
/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Carles Vela, Martí Bolet
   Contact: support.idi@movvo.eu
*/


#include "agl_imu/imu_drivers/wit_imu.hpp"

const double PI = 3.14159265;

namespace agl_imu{
namespace imu_drivers{

/*****************************************************************************
                    IMU NODE CONSTRUCTOR
*****************************************************************************/
//================================================
WitImu::WitImu(std::string driver_name) : DriverAbstract(driver_name)
//================================================
{

}

WitImu::~WitImu()
{
    // Disconnect();
}

//================================================
void WitImu::Initialize()
//================================================
{
    // nh_->declare_parameter(driver_name_+".frame_id", "imu_link");
	nh_->declare_parameter(driver_name_+".frame_id", "base_link");
	// The serial port could change randomly, see udev rules for different usb devices or a workaround with the existing code.
	nh_->declare_parameter(driver_name_+".serial_port", "/dev/ttyUSB0");
	nh_->declare_parameter(driver_name_+".baudrate", 115200);
}

//================================================
bool WitImu::Configure() 
//================================================
{
    RCLCPP_INFO(nh_->get_logger(), "Configuring WIT_IMU");
	// Retrieving parameters of this node
	try {
		// Get parameters
		frame_id_ = nh_->get_parameter(driver_name_+".frame_id").as_string();
        serial_port_ = nh_->get_parameter(driver_name_+".serial_port").as_string();
        baudrate_ = nh_->get_parameter(driver_name_+".baudrate").as_int();
		
		RCLCPP_INFO(nh_->get_logger(),"WIT_IMU parameters initialized");
	}
    catch (rclcpp::exceptions::InvalidParameterValueException& ex){
        RCLCPP_ERROR(nh_->get_logger(),"Error init parameters in WIT_IMU DRIVER, InvalidParameterValueException");
        std::cout << ex.what() << std::endl;
        sm_->updateErrorCritical(BAD_PARAMETERS, BAD_PARAMETERS_DESC);
        return false;
    }
    catch (std::runtime_error& ex){
        RCLCPP_ERROR(nh_->get_logger(),"Error init parameters in WIT_IMU DRIVER, runtime_error");
        std::cout << ex.what() << std::endl;
        sm_->updateErrorCritical(BAD_PARAMETERS, BAD_PARAMETERS_DESC);
        return false;
    }
	catch (...) {
		return false;
    }

	try {
		// Initialize ROS2
		calibration_srv_ = nh_->create_service<std_srvs::srv::Empty>("~/calibrate", std::bind(&WitImu::calibrateAcceleration, this, _1, _2));
    }
    catch(...) {
        return false;
    }
	return true;
}

// Method to open the connection
//================================================
bool WitImu::Connect()
//================================================
{
// Try to open port
	if (!openSerial(serial_port_, baudrate_)) {
		RCLCPP_ERROR(nh_->get_logger(),"Error opening IMU serial port in USB0");
		return false;
	}
	return true;
}

//================================================
sensor_msgs::msg::Imu WitImu::GetImuMsg()
//================================================
{
	std::vector<float> AngVel (3, 0.0);
	std::vector<float> AngAcc (3, 0.0);
	std::vector<float> Angles (3, 0.0);

	// Read Angles
	Angles = getAngles();
	// euler angles to quaternion
	tf2::Quaternion q;	
	q.setRPY(Angles[0]*PI/180, Angles[1]*PI/180, Angles[2]*PI/180);

	// Read Angular Vel Data
	AngVel = getAngVel();

	// Read Angular Accel Data
	AngAcc = getAngAcc();

	// Fill IMU messge
	auto message = sensor_msgs::msg::Imu();
	message.header.stamp = nh_->now();
	message.header.frame_id = frame_id_;
	message.orientation.x = q.x();
	message.orientation.y = q.y();
	message.orientation.z = q.z();
	message.orientation.w = q.w();
	message.orientation_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

	message.angular_velocity.x = AngVel[0];
	message.angular_velocity.y = AngVel[1];
	message.angular_velocity.z = AngVel[2];
	message.angular_velocity_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

	message.linear_acceleration.x = AngAcc[0];
	message.linear_acceleration.y = AngAcc[1];
	message.linear_acceleration.z = AngAcc[2];
	message.linear_acceleration_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

	// imu_pub_->publish(message);
	return message;
}

//================================================
bool WitImu::ErrorHandle()
//================================================
{
	if (sm_->critical & BAD_PARAMETERS){
        if(Configure()) {
            return true;
        }
    }
	else if (sm_->critical & BAD_BAUDRATE || sm_->critical & CANT_SET_BAUDRATE || sm_->critical & WRONG_BAUDRATE ||
				sm_->critical & CANT_OPEN_PORT || sm_->critical & CANT_OPEN_PORT_NO_PERM || sm_->critical & UNKNOWN_ERROR){
		if(openSerial(serial_port_, baudrate_)){
			return true;
		}
	}
    return false;
}

/*****************************************************************************
                FUNCIONES PROPIAS FUNCIONAMIENTO NODO
*****************************************************************************/
//================================================
bool WitImu::openSerial(std::string port, int baudrate)
//================================================
{
	// Baud rate
	LibSerial::BaudRate baud;
	try {
		baud = baudrateOf(baudrate);
		sm_->updateSolvedCritical(BAD_BAUDRATE, BAD_BAUDRATE_DESC);
	} catch(...) {
		sm_->updateErrorCritical(BAD_BAUDRATE, BAD_BAUDRATE_DESC);
		return false;
	}

	// Open port
 	try {
		// If serial is already opened
		if (serial_is_opened_) {
			// Close acutal serial
			closeSerial();
		}
		// And reopen 
		serial_.Open(port);        
    } catch (...) {
		if (checkIfSerial(port)) {
			sm_->updateErrorCritical(CANT_OPEN_PORT_NO_PERM, CANT_OPEN_PORT_NO_PERM_DESC);
			sm_->updateSolvedCritical(CANT_OPEN_PORT, CANT_OPEN_PORT_DESC);
		} else {
			sm_->updateErrorCritical(CANT_OPEN_PORT, CANT_OPEN_PORT_DESC);
			sm_->updateSolvedCritical(CANT_OPEN_PORT_NO_PERM, CANT_OPEN_PORT_NO_PERM_DESC);
		}
		return false;
    }
	
	// Configure BaudRate of port
	try {
		serial_.SetBaudRate(baud);
	} catch(...) {
		sm_->updateErrorCritical(CANT_SET_BAUDRATE, CANT_SET_BAUDRATE_DESC);
		return false;
	}

	// Check if open
    if(serial_.IsOpen()){
		RCLCPP_INFO(nh_->get_logger(), "Serial port opened '%s' at baudrate %d", serial_port_.c_str(), baudrate_);
		serial_is_opened_ = true;
		sm_->updateSolvedCritical(CANT_OPEN_PORT, CANT_OPEN_PORT_DESC);
		sm_->updateSolvedCritical(CANT_OPEN_PORT_NO_PERM, CANT_OPEN_PORT_NO_PERM_DESC);
		sm_->updateSolvedCritical(UNKNOWN_ERROR, UNKOWN_ERROR_DESC);
    } else {
		sm_->updateErrorCritical(UNKNOWN_ERROR, UNKOWN_ERROR_DESC);
		return false;
    }

	// Test if correct BaudRate
	if (baudrate_ != readBaudRate()) {
		sm_->updateErrorCritical(WRONG_BAUDRATE, WRONG_BAUDRATE_DESC);
		return false;
	} else {
		sm_->updateSolvedCritical(WRONG_BAUDRATE, WRONG_BAUDRATE_DESC);
	}

	return true;
}

// Esta función sirve para comprobar si hay un joystick conectado.
//================================================
bool WitImu::checkIfSerial(std::string serial)
//================================================
{
	std::vector <std::string> path_of_port;
	boost::split(path_of_port, serial, boost::algorithm::is_any_of("/"));

    std::string inputs = atlas_utils::utils::executeCommand("ls /dev/");

	if (boost::algorithm::contains(inputs, path_of_port[path_of_port.size()-1])) {
		return true;
	}
    return false;
}

//================================================
int WitImu::readBaudRate()
//================================================
{
	std::vector<uint8_t> query = {0x50, 0x03, 0x00, 0x04, 0x00, 0x01};

	// Calculate CRC
	uint16_t crc = ModRTU_CRC(query);
	query.push_back(crc & 0xFF);
	query.push_back(crc >> 8);

    serial_.Write(query);

	std::vector<uint8_t> read_data;
	try {
		serial_.Read(read_data, 7, 100);
		sm_->updateSolvedWarning(TIMEOUT_READING, TIMEOUT_READING_DESC);
	} catch(LibSerial::ReadTimeout &) {
		sm_->updateErrorWarning(TIMEOUT_READING, TIMEOUT_READING_DESC);
		return 0;
	}

	uint16_t baud = (uint16_t)(read_data[3]<<8 ^ read_data[4]);
	switch (baud) {
		case 1:
			return 4800;
		case 2:
			return 9600;
		case 3:
			return 19200;
		case 4:
			return 38400;
		case 5:
			return 57600;
		case 6:
			return 115200;
		case 7:
			return 230400;
		case 8:
			return 460800;
		case 9:
			return 921600;
		default:
			return 0;
			break;
	}

	return 0;
}

//================================================
LibSerial::BaudRate WitImu::baudrateOf(int baudrate)
//================================================
{
	switch (baudrate) {
		case 50:
			return LibSerial::BaudRate::BAUD_50;
		case 75:
			return LibSerial::BaudRate::BAUD_75;
		case 110:
			return LibSerial::BaudRate::BAUD_110;
		case 134:
			return LibSerial::BaudRate::BAUD_134;
        case 150:
			return LibSerial::BaudRate::BAUD_150;
        case 200:
			return LibSerial::BaudRate::BAUD_200;
        case 300:
			return LibSerial::BaudRate::BAUD_300;
        case 600:
			return LibSerial::BaudRate::BAUD_600;
        case 1200:
			return LibSerial::BaudRate::BAUD_1200;
        case 1800:
			return LibSerial::BaudRate::BAUD_1800;
        case 2400:
			return LibSerial::BaudRate::BAUD_2400;
        case 4800:
			return LibSerial::BaudRate::BAUD_4800;
        case 9600:
			return LibSerial::BaudRate::BAUD_9600;
        case 19200:
			return LibSerial::BaudRate::BAUD_19200;
        case 38400:
			return LibSerial::BaudRate::BAUD_38400;
        case 57600:
			return LibSerial::BaudRate::BAUD_57600;
        case 115200:
			return LibSerial::BaudRate::BAUD_115200;
        case 230400:
			return LibSerial::BaudRate::BAUD_230400;
        case 460800:
			return LibSerial::BaudRate::BAUD_460800;
        case 500000:
			return LibSerial::BaudRate::BAUD_500000;
        case 576000:
			return LibSerial::BaudRate::BAUD_576000;
        case 921600:
			return LibSerial::BaudRate::BAUD_921600;
        case 1000000:
			return LibSerial::BaudRate::BAUD_1000000;
        case 1152000:
			return LibSerial::BaudRate::BAUD_1152000;
        case 1500000:
			return LibSerial::BaudRate::BAUD_1500000;
		default:
			throw "Invalid BaudRate";
			break;
	}
}


//================================================
void WitImu::closeSerial()
//================================================
{
	// If serial port is opened
	if (serial_is_opened_) {
		serial_.Close();
		serial_is_opened_ = false;
	}
}

// Compute the MODBUS RTU CRC
//================================================
uint16_t WitImu::ModRTU_CRC(std::vector<uint8_t> buf)
//================================================
{
  	unsigned int crc = 0xFFFF;
	for (unsigned int pos = 0; pos < buf.size(); pos++) {
		crc ^= (uint16_t)buf[pos];       // XOR byte into least sig. byte of crc

		for (int i = 8; i != 0; i--) {   // Loop over each bit
      		if ((crc & 0x0001) != 0) {   // If the LSB is set
        		crc >>= 1;               // Shift right 
        		crc ^= 0xA001;			 // and XOR 0xA001
			} else {                     // Else LSB is not set
        		crc >>= 1;               // Just shift right
			}
    	}
	}

	// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  	return crc;
}

//================================================
void WitImu::calibrateAcceleration(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
									std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
//================================================
{
	// Send unlock command
	std::vector<uint8_t> unlock_command = {0x50, 0x06, 0x00, 0x69, 0xB5, 0x88};
	writeCommand(unlock_command);

	// Set the CSW register to acceleration calibration mode
	writeCommand({0x50, 0x06, 0x00, 0x01, 0x00, 0x01});

	// Wait for 3 to 5 seconds withoyut moving the sensor
	std::this_thread::sleep_for(4.5s);

	// Set the CSW register to normal mode
	writeCommand({0x50, 0x06, 0x00, 0x01, 0x00, 0x00});

	// Send "Save Configuration" command
	writeCommand({0x50, 0x06, 0x00, 0x00, 0x00, 0x00});
}

//================================================
std::vector<double> WitImu::readAccelerationBias()
//================================================
{
	std::vector<uint8_t> write_query = {0x50, 0x03, 0x00, 0x05, 0x00, 0x03};

	std::vector<int> tmp = writeQuery(write_query);

	return {(double)tmp[0]*16*9.8/32768, (double)tmp[1]*16*9.8/32768, (double)tmp[2]*16*9.8/32768};
}

//================================================
std::vector<float> WitImu::readAngularVelocityBias()
//================================================
{
	std::vector<uint8_t> write_query = {0x50, 0x03, 0x00, 0x08, 0x00, 0x03};

	std::vector<int> tmp = writeQuery(write_query);

	return {(float)tmp[0]*2000/32768, (float)tmp[1]*2000/32768, (float)tmp[2]*2000/32768};
}

//================================================
std::vector<float> WitImu::getAngles()
//================================================
{
	std::vector<uint8_t> write_query = 
				{0x50, 0x03, 0x00, 0x3D, 0x00, 0x03};

	std::vector<float> AngleXYZ;
	std::vector<int> tmp = writeQuery(write_query);

	for (uint64_t i=0 ; i < tmp.size(); i++) {
		AngleXYZ.push_back((float)tmp[i]*180/32768 );
	}

	return AngleXYZ;
}

//================================================
std::vector<float> WitImu::getAngVel()
//================================================
{
	std::vector<uint8_t> write_query = 
				{0x50, 0x03, 0x00, 0x37, 0x00, 0x03};

	std::vector<float> VelocityXYZ;
	std::vector<int> tmp = writeQuery(write_query);

	for (uint64_t i=0 ; i < tmp.size();i++) {
		VelocityXYZ.push_back((float)tmp[i]*2000/32768);
	}

	return VelocityXYZ;
}

//================================================
std::vector<float> WitImu::getAngAcc()
//================================================
{
	std::vector<uint8_t> write_query = 
				{0x50, 0x03, 0x00, 0x34, 0x00, 0x03};

	std::vector<float> AccelXYZ;
	std::vector<int> tmp = writeQuery(write_query);

	for (uint64_t i=0 ; i < tmp.size();i++) {
		AccelXYZ.push_back((float)tmp[i]*16*9.8/32768);
	}

	return AccelXYZ;
}

//================================================
void WitImu::writeCommand(std::vector<uint8_t> command, bool read_response)
//================================================
{
	// Calculate CRC
	uint16_t crc = ModRTU_CRC(command);
	command.push_back(crc & 0xFF);
	command.push_back(crc >> 8);


    serial_.Write(command);
	
	if (read_response) {
		std::vector<uint8_t> read_data;
		serial_.Read(read_data, 8, 100);
	}
}

//================================================
std::vector<int> WitImu::writeQuery(std::vector<uint8_t> query)
//================================================
{
	std::vector<uint8_t> read_data;

	// Calculate CRC
	uint16_t crc = ModRTU_CRC(query);
	query.push_back(crc & 0xFF);
	query.push_back(crc >> 8);

	std::vector<int> tmp (3,0.0);
		
	serial_.Write(query);
	try {
		serial_.Read(read_data, 11, 100);
		sm_->updateSolvedWarning(TIMEOUT_READING, TIMEOUT_READING_DESC);
	} catch(LibSerial::ReadTimeout &) {
		sm_->updateErrorWarning(TIMEOUT_READING, TIMEOUT_READING_DESC);
		return {0, 0, 0};
	}

	tmp[0] = (int16_t)(read_data[3]<<8 ^ read_data[4]);
	tmp[1] = (int16_t)(read_data[5]<<8 ^ read_data[6]);
	tmp[2] = (int16_t)(read_data[7]<<8 ^ read_data[8]);

	return tmp;
}

} // namespace imu_drivers
} // namespace agl_imu
