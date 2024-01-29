/*
 *  Copyright 2023 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#ifndef IMAGE_TRANSPORT_HPP
#define IMAGE_TRANSPORT_HPP

// C++ Standard
#include <iostream>
#include <string>
#include <vector>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "image_transport/transport_hints.hpp"
#include "image_transport/camera_subscriber.hpp"
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;
using CompressedImage = sensor_msgs::msg::CompressedImage;

namespace atlas_utils {
namespace image {

class ImageTransport {
    public:
    ImageTransport(rclcpp::Node::SharedPtr nh_);

    // Descomprimir imagenes
    void create_compressed_subs(std::string topic_name);
    void CompressedCallback(const CompressedImage::SharedPtr img);

    // Comprimit imagenes
    void create_compressed_pub(std::string topic_name);
    void publish_compressed_image(cv::Mat img);
    // Getters
    cv::Mat get_img();
    protected:

    private:
    rclcpp::Node::SharedPtr nh;

    // Variables para las funciones de descomprimir
    cv::Mat recieved_image;
    sensor_msgs::msg::Image image_msg;
    rclcpp::Subscription<CompressedImage>::SharedPtr img_subs;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_repub;

    // Variables para las funciones de compression
    rclcpp::Publisher<CompressedImage>::SharedPtr img_pub;
    CompressedImage comp_img_msg;

};

} // namespace image
} // namespace atlas_utils

#endif // IMAGE_TRANSPORT_HPP