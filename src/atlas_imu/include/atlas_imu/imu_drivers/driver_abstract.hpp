/*
   Copyright 2022 @ Movvo - Robotics
   ---------------------------------------------------------
   Authors: María Mercadé
   Contact: support.idi@movvo.eu
*/

#ifndef ATLAS_IMU_DRIVER_ABSTRACT_HPP
#define ATLAS_IMU_DRIVER_ABSTRACT_HPP

/*
    Abstract class for generic Imu drivers
*/

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <atlas_imu/imu_interface.hpp>
#include "atlas_utils/sm/StateMachine.hpp"

namespace atlas_imu {
namespace imu_drivers {

class DriverAbstract : public ImuInterface {
    public:
        // Constructor
        DriverAbstract(std::string driver_name) : driver_name_(driver_name) {};
        // Descructor
        virtual ~DriverAbstract() {};

        // Setter of node
        void SetNode(std::shared_ptr<rclcpp::Node> nh) { nh_ = nh; };

        // Setter of state machine
        void setSM(std::shared_ptr<atlas_utils::sm::StateMachine> sm) { sm_ = sm; };

        // Method to initialize parameters
        virtual void Initialize() {};

        // Method to configure the driver
        virtual bool Configure() { return true; };

        // Method to open the connection
        virtual bool Connect() { return true; };

        // Method to check if driver is connected
        // bool IsConnected() { return connected_; };

        // // Method to close the connection
        // virtual void Disconnect() {};

        // Publish IMU msgs through ROS2 topic
        virtual sensor_msgs::msg::Imu GetImuMsg() { return imu; };

        // Get IMU orientation(angles)
        virtual std::vector<float> getAngles() { return {}; };

        // Get IMU angular velocity
        virtual std::vector<float> getAngVel() { return {}; };

        // Get IMU linear accelerations
        virtual std::vector<float> getAngAcc() { return {}; };
        
        // Try to solve errors if posible
        virtual bool errorHandle() { return true; };

        // Getter of driver name
        std::string GetName() { return driver_name_; };

    protected:
        // Attributes
        // bool connected_{false};
        std::string driver_name_;
        std::shared_ptr<rclcpp::Node> nh_;
        std::shared_ptr<atlas_utils::sm::StateMachine> sm_;
        sensor_msgs::msg::Imu imu;
};

} // namespace imu_drivers
} // namespace atlas_imu

#endif // ATLAS_IMU_DRIVER_ABSTRACT_HPP