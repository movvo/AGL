/*
   Copyright 2022 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: María Mercadé de Luna
   Contact: support.idi@movvo.eu
*/

#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

//C++ Standard 
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream> 
#include <vector>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// //Own interfaces  
#include "atlas_utils/sm/StateMachine.hpp"
#include "atlas_utils/general/system_funcs.hpp"

// IMU Interface
#include "agl_imu/imu_interface.hpp"
#include "imu_errors.hpp"
#include "agl_imu/imu_drivers/driver_abstract.hpp"

using namespace std::chrono_literals;

namespace agl_imu {

class ImuNode : public rclcpp::Node, public atlas_utils::sm::StateMachine{
    public:
        ImuNode();
        virtual ~ImuNode();

        /*****************************
         *  FUNCIONES ESTADO DEL NODO
        ******************************/
        void Unconfigured();
        void StandBy();
        void Run();
        void Fault();
        void ShutDown();

        /*****************************
         *  FUNCIONES PROPIAS DEL NODO
        ******************************/
        void InitializeImu();
        bool ConfigureImu();
        bool ConnectImu();
        void Release();
        void PublishImuInfo();
        void SetDriver(std::shared_ptr<imu_drivers::DriverAbstract> driver);

    private:
        //Parameters
        struct Parameters{
            rclcpp::Parameter autorun;
            rclcpp::Parameter imu_topic;
        };
        struct Parameters parameters_;

        std::shared_ptr<rclcpp::Node> nh_;
        std::shared_ptr<atlas_utils::sm::StateMachine> sm_;
        std::shared_ptr<imu_drivers::DriverAbstract> driver_;

        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; 

        /*****************************
         *  RECONFIGURACIÓN DINÁMICA
        ******************************/
        rcl_interfaces::msg::SetParametersResult dyn_reconf_callback(const std::vector<rclcpp::Parameter> & parameters);
};

} // namespace agl_imu

#endif // IMU_NODE_HPP