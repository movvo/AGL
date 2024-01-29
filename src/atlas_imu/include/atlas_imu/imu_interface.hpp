/*
   Copyright 2022 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: María Mercadé De Luna
   Contact: support.idi@movvo.eu
*/

#ifndef IMU_INTERFACE_HPP
#define IMU_INTERFACE_HPP

#include <memory>
#include <vector>

namespace atlas_imu {

class ImuInterface {
    public:

        virtual ~ImuInterface() {};

        // Method to declare parameters
        virtual void Initialize() = 0;

        // Method to configure the class and get parameters
        virtual bool Configure() = 0;

        // Method to connect to imu
        virtual bool Connect() = 0;

        // // Method to check if driver is connected
        // virtual bool IsConnected() = 0;

        // // Method to close the connection
        // virtual void Disconnect() = 0;

        // Publish IMU msgs through ROS2 topic
        virtual sensor_msgs::msg::Imu GetImuMsg() = 0;

        // Get IMU orientation(angles)
        virtual std::vector<float> getAngles() = 0;

        // Get IMU angular velocity
        virtual std::vector<float> getAngVel() = 0;

        // Get IMU linear accelerations
        virtual std::vector<float> getAngAcc() = 0;
        
        // Try to solve errors if posible
        virtual bool ErrorHandle() = 0;

        // Getter of driver name
        virtual std::string GetName() = 0;
};

} // namespace atlas_imu

#endif // IMU_INTERFACE_HPP 