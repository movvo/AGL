/*
   Copyright 2022 @ AGEVE
   ---------------------------------------------------------
   Authors: María Mercadé
   Contact: support.idi@movvo.eu
*/


#include "atlas_imu/imu_drivers/aceinna_imu.hpp"

using std::placeholders::_1;

const float PI = 3.14159265;

namespace atlas_imu {
namespace imu_drivers {

//================================================
AceinnaImu::AceinnaImu(std::string driver_name) : DriverAbstract(driver_name)
//================================================
{

}

AceinnaImu::~AceinnaImu()
{
    // Disconnect();
}

// Method to configure the driver
//================================================
void AceinnaImu::Initialize()
//================================================
{
    nh_->declare_parameter(driver_name_+".frame_id", "imu_link");
    nh_->declare_parameter(driver_name_+".receive_topic", "CanComms/CAN2IMU");
    nh_->declare_parameter(driver_name_+".timeout", 500000000);

    can_db_ = can_utils::CANdb();
}

//================================================
bool AceinnaImu::Configure() 
//================================================
{
    RCLCPP_INFO(nh_->get_logger(), "Configuring Aceinna IMU");
	// Retrieving parameters of this node
	try {
		// Get parameters
        frame_id_ = nh_->get_parameter(driver_name_+".frame_id").as_string();
        receive_topic_ = nh_->get_parameter(driver_name_+".receive_topic").as_string();
        timeout_ = nh_->get_parameter(driver_name_+".timeout").get_value<uint32_t>();

		sm_->updateSolvedCritical(BAD_PARAMETERS, BAD_PARAMETERS_DESC);
		RCLCPP_INFO(nh_->get_logger(),"Aceinna IMU parameters initialized");
	}
    catch (rclcpp::exceptions::InvalidParameterValueException& ex){
        RCLCPP_ERROR(nh_->get_logger(),"Error init parameters in Aceinna IMU DRIVER, InvalidParameterValueException");
        std::cout << ex.what() << std::endl;
        sm_->updateErrorCritical(BAD_PARAMETERS, BAD_PARAMETERS_DESC);
        return false;
    }
    catch (std::runtime_error& ex){
        RCLCPP_ERROR(nh_->get_logger(),"Error init parameters in Aceinna IMU DRIVER, runtime_error");
        std::cout << ex.what() << std::endl;
        sm_->updateErrorCritical(BAD_PARAMETERS, BAD_PARAMETERS_DESC);
        return false;
    }
    catch (...) {
		return false;
    }

    // Create Publisher and subscriber
    try {
        CAN2IMU_sub_ = nh_->create_subscription<atlas_interfaces::msg::CanMsgs>(receive_topic_, 10, std::bind(&AceinnaImu::ros2imu_callback, this, _1));
    }
    catch(...) {
        return false;
    }
    
    imu_subs_time_ = nh_->now();
    return true;
}

// Method to open the connection
//================================================
bool AceinnaImu::Connect()
//================================================
{
	return true;
}

//================================================
sensor_msgs::msg::Imu AceinnaImu::GetImuMsg() 
//================================================
{
    std::vector<float> AngVel (3, 0.0);
	std::vector<float> AngAcc (3, 0.0);
	std::vector<float> Angles (3, 0.0);

	// Read Angles
	Angles = getAngles();
	// Euler angles to quaternion
	tf2::Quaternion q;	
	q.setRPY(Angles[0], Angles[1], Angles[2]);

	// Read Angular Vel Data
	AngVel = getAngVel();

	// Read Angular Accel Data
	AngAcc = getAngAcc();
    
    auto imu_msg = sensor_msgs::msg::Imu();

    // Refresh timestamp
    imu_msg.header.stamp = nh_->now();
    imu_msg.header.frame_id = frame_id_;

    // Read Can Comm variables
    // Orientation
    imu_msg.orientation.x = q.x();
	imu_msg.orientation.y = q.y();
	imu_msg.orientation.z = q.z();
	imu_msg.orientation.w = q.w();

    imu_msg.orientation_covariance = {0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05}; // INVENTADA


    // Angular velocity
    imu_msg.angular_velocity.x = AngVel[0];
    imu_msg.angular_velocity.y = AngVel[1];
    imu_msg.angular_velocity.z = AngVel[2];

    imu_msg.angular_velocity_covariance = {0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05}; // INVENTADA

    // Linear acceleration
    imu_msg.linear_acceleration.x = AngAcc[0];
	imu_msg.linear_acceleration.y = AngAcc[1];
	imu_msg.linear_acceleration.z = AngAcc[2];

    imu_msg.linear_acceleration_covariance = {0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05}; // INVENTADA
    // Publish the IMU msg

    // check for timeout
    if ((nh_->now()-imu_subs_time_).nanoseconds() > timeout_){
        sm_->updateErrorCritical(IMU_TIMEOUT,IMU_TIMEOUT_DESC);
        RCLCPP_ERROR(nh_->get_logger(),"Timeout, not receiving IMU data from cancoms");
    }
    else {
        sm_->updateSolvedCritical(IMU_TIMEOUT,IMU_TIMEOUT_DESC);
    }
    return imu_msg;
}

//================================================
bool AceinnaImu::ErrorHandle() 
//================================================
{
    if (sm_->critical & BAD_PARAMETERS){
        if (Configure()){
            sm_->updateSolvedCritical(BAD_PARAMETERS, BAD_PARAMETERS_DESC);
            return true;
        }
    }
    return false;
}

//================================================
std::vector<float> AceinnaImu::getAngles()
//================================================
{
    // Roll, pitch in degrees and yaw in radians
    float roll = can_db_.Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Roll;
    float pitch = can_db_.Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Pitch;
    float yaw = can_db_.Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Yaw;

	std::vector<float> AngleXYZ = {roll,pitch,yaw};

	return AngleXYZ;
}

//================================================
std::vector<float> AceinnaImu::getAngVel()
//================================================
{
	// Angular velocities in deg/s
    float ang_vel_x = can_db_.Aceinna_Rate_struct->signal_value.Aceinna_GyroX;
    float ang_vel_y = can_db_.Aceinna_Rate_struct->signal_value.Aceinna_GyroY;
    float ang_vel_z = can_db_.Aceinna_Rate_struct->signal_value.Aceinna_GyroZ;
    
    // Angular velocities in rad/s
    std::vector<float> VelocityXYZ = {ang_vel_x*PI/180,ang_vel_y*PI/180,ang_vel_z*PI/180};
	

	return VelocityXYZ;
}

//================================================
std::vector<float> AceinnaImu::getAngAcc()
//================================================
{
	// Linear accelerations in m/s²
    float AccX = can_db_.Aceinna_Accel_struct->signal_value.Aceinna_AccX;
    float AccY = can_db_.Aceinna_Accel_struct->signal_value.Aceinna_AccY;
    float AccZ = can_db_.Aceinna_Accel_struct->signal_value.Aceinna_AccZ;

	std::vector<float> AccelXYZ = {AccX, AccY, AccZ};

	return AccelXYZ;
}

//================================================
void AceinnaImu::ros2imu_callback(atlas_interfaces::msg::CanMsgs::SharedPtr msgs)
//================================================
{
    RCLCPP_DEBUG(nh_->get_logger(), "CAN IMU MESSAGE RECEIVED");
    imu_subs_time_ = nh_->now(); 
    sm_->updateSolvedCritical(IMU_TIMEOUT,IMU_TIMEOUT_DESC);
    for (uint8_t i = 0; i<msgs->msgs.size(); i++){
        can_utils::CANdb::CAN_msg msg;
        msg.id = msgs->msgs[i].id;
        msg.size = msgs->msgs[i].size;
        msg.timestamp_us = msgs->msgs[i].can_stamp;
        msg.data = msgs->msgs[i].data;
        can_db_.CANRx_Read(msg);
    }
}

} // namespace imu_drivers
} // namespace atlas_imu
