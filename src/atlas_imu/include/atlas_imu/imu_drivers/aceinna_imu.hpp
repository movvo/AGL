/*
   Copyright 2022 @ AGEVE
   ---------------------------------------------------------
   Authors: María Mercadé
   Contact: support.idi@movvo.eu
*/
#ifndef ACEINNA_IMU_HPP_
#define ACEINNA_IMU_HPP_

//C++ Standard
#include <memory>
#include <string>
#include "tf2/LinearMath/Quaternion.h"

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Own interfaces
#include "atlas_interfaces/msg/can_msgs.hpp"

// Others
#include "atlas_imu/imu_drivers/driver_abstract.hpp"
#include "atlas_imu/imu_errors.hpp"
#include "atlas_imu/can_utils/CANdb.hpp"
// #include "atlas_utils/sm/StateMachine.hpp"

// using std::placeholders::_1;
// using std::placeholders::_2;
using namespace std::chrono_literals;

namespace atlas_imu {
namespace imu_drivers {

//================================================
class AceinnaImu : public DriverAbstract
//================================================
{
  public:
    AceinnaImu(std::string driver_name);
    ~AceinnaImu();

    // Method to configure parameters
    void Initialize(); 

    // Configuration of ACEINNA IMU
    bool Configure();

    // Method to open the connection
    bool Connect();

    // // Method to close the connection
    // void Disconnect();

    // Publish IMU info through ROS2 topic
    sensor_msgs::msg::Imu GetImuMsg();

    // Get IMU orientation(angles)
    std::vector<float> getAngles();

    // Get IMU angular velocity
    std::vector<float> getAngVel();

    // Get IMU linear accelerations
    std::vector<float> getAngAcc();

    // Consider errors
    bool ErrorHandle();

  private:
    // Parameter
    std::string frame_id_;
    std::string receive_topic_;
    uint32_t timeout_;
    // CAN utils
    can_utils::CANdb can_db_;

    // Subscriber
    rclcpp::Subscription<atlas_interfaces::msg::CanMsgs>::SharedPtr CAN2IMU_sub_;

    void ros2imu_callback(atlas_interfaces::msg::CanMsgs::SharedPtr msgs);

    rclcpp::Time imu_subs_time_;
};

} // namespace imu_drivers
} // namespace atlas_imu

#endif // ACEINNA_IMU_HPP_