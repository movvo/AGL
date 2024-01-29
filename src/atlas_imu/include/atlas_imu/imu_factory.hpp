/*
   Copyright 2022 @ AGEVE
   ---------------------------------------------------------
   Authors: María Mercadé De Luna
   Contact: support.idi@movvo.eu
*/

#ifndef IMU_FACTORY_HPP
#define IMU_FACTORY_HPP

#include <string>
#include <iostream>
#include "atlas_imu/imu.hpp"
#include "atlas_imu/imu_drivers/wit_imu.hpp"
#include "atlas_imu/imu_drivers/aceinna_imu.hpp"

namespace atlas_imu {

class ImuFactory {
    public:
        inline static std::shared_ptr<ImuNode> create() {
            auto options = rclcpp::NodeOptions().arguments({"--ros-args", "-r", std::string("__node:=ImuFactory"), "--"});
            auto node = std::make_shared<rclcpp::Node>("ImuFactory", options);
            
            // IMU
            std::shared_ptr<ImuNode> imu_node = std::make_shared<ImuNode>();

            node->declare_parameter("model", "ACEINNA_IMU");
            try {
                std::string model = node->get_parameter("model").get_value<std::string>();

                // WIT_IMU
                if (model == "WIT_IMU") {
                    imu_node->SetDriver(std::make_shared<imu_drivers::WitImu>(model));
                } 
                // ACEINNA_IMU
                else if(model == "ACEINNA_IMU"){
                    imu_node->SetDriver(std::make_shared<imu_drivers::AceinnaImu>(model));
                } 
                else {
                    throw std::runtime_error("Invalid 'model' in IMU Factory '" + model + "'");
                }
            } catch (std::runtime_error & ex) {
                RCLCPP_ERROR(node->get_logger(),"%s", ex.what());
                exit(-1);
            }
            
            return imu_node;
        }
};

} // namespace atlas_imu

#endif // IMU_FACTORY_HPP 