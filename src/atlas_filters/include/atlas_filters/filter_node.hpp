/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Albert Arlà Romero
   Contact: support.idi@ageve.net
*/

#ifndef FILTER_NODE_HPP
#define FILTER_NODE_HPP

//C++ Standard 
#include <chrono>
#include <string>
#include <iostream> 
#include <vector>
#include <math.h> 
#include <Eigen/Geometry> 

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>  
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#define SENSOR_MSGS_SKIP_WARNING
#include <sensor_msgs/point_cloud_conversion.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp> 

#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace atlas_filters {

class FilterNode : public rclcpp::Node {
    public: 
        FilterNode();
        virtual ~FilterNode();


        /*****************************
         *  VARIABLES PÚBLICAS
        ******************************/

    private:

        void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud, const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_info);

        /** Modified function from package::pointcloud_to_laserscan: pointcloud_to_laserscan_node.cpp **/
        void PointCloudToLaserScan(sensor_msgs::msg::PointCloud2 & cloud_msg, sensor_msgs::msg::LaserScan & laser_msg);

        /*****************************
         *  ROS2
        ******************************/
        std::shared_ptr<rclcpp::Node> rclcpp_node_;        

        message_filters::Subscriber<sensor_msgs::msg::LaserScan> subscriber_laser1;
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> subscriber_laser2;
        // std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>> laser_sync_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> approximate_policy;
        std::shared_ptr<message_filters::Synchronizer<approximate_policy>> laser_sync_;

        void LaserSyncCallback( const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg_1,
                                const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg_2) ;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;

        sensor_msgs::msg::LaserScan laserscan_msg;

        /*****************************
         *  RECONFIGURACIÓN DINÁMICA
        ******************************/
        rcl_interfaces::msg::SetParametersResult dyn_reconf_callback(const std::vector<rclcpp::Parameter> & parameters);

        /*****************************
         *  VARIABLES PRIVADAS
        ******************************/
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_laser_front;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_laser_back;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_laser_front;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_laser_back;

        laser_geometry::LaserProjection projector_front_;
        laser_geometry::LaserProjection projector_back_;

        //Parameters
        struct Parameters{
            rclcpp::Parameter laser_topic1;
            rclcpp::Parameter laser_topic2;
            rclcpp::Parameter frequency;
            rclcpp::Parameter angle_min;
            rclcpp::Parameter angle_max;
        };
        struct Parameters parameters;
};

} // namespace atlas_filters

#endif //FILTER_NODE_HPP