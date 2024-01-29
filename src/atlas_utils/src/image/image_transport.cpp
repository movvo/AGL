/*
 *  Copyright 2023 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#include "atlas_utils/image/image_transport.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using namespace atlas_utils::image;
using CompressedImage = sensor_msgs::msg::CompressedImage;


ImageTransport::ImageTransport(rclcpp::Node::SharedPtr nh_){
    nh = nh_;



}

void ImageTransport::create_compressed_subs(std::string topic_name){
    // Create subscription to compressed img
    img_subs = nh->create_subscription<CompressedImage>(topic_name+"/compressed", 10, std::bind(&ImageTransport::CompressedCallback, this, _1));
    // Create republish of the img
    img_repub = nh->create_publisher<sensor_msgs::msg::Image>(topic_name+"/raw",10);
}

void ImageTransport::create_compressed_pub(std::string topic_name)
{
  img_pub = nh->create_publisher<CompressedImage>(topic_name+"/compressed",10);
}

// Get a cv::Mat imgae (OpenCV image) compress it and publish
void ImageTransport::publish_compressed_image(cv::Mat img)
{
  std::vector<uchar> buf;
  if (cv::imencode(".jpg", img, buf)){
    std_msgs::msg::Header header;
    header.stamp  = nh->now();  
    comp_img_msg.header = header;
    comp_img_msg.format = ".jpg";
    comp_img_msg.data = buf;
    img_pub->publish(comp_img_msg);
  }
  else{
    RCLCPP_WARN(nh->get_logger(),"Can't compress image");
  }
}



void ImageTransport::CompressedCallback(const CompressedImage::SharedPtr img)
{
  try
  {
    // get uncompressed image as cv::Mat
    recieved_image = cv::imdecode(cv::Mat(img->data),1);//convert compressed image data to cv::Mat
    // get sensor_msgs Image from cv::Mat
    cv_bridge::CvImage img_bridge;
    std_msgs::msg::Header header; // empty header
    header.stamp = nh->now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, recieved_image);
    img_bridge.toImageMsg(image_msg); // from cv_bridge to sensor_msgs::Image
    img_repub->publish(image_msg);

  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_WARN(nh->get_logger(),"Could not convert to image!");
  }
}

cv::Mat ImageTransport::get_img(){
    return recieved_image;
}


