/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Albert Arlà Romero
   Contact: support.idi@ageve.net
*/

#include "atlas_filters/filter_node.hpp"

namespace atlas_filters {

//================================================
FilterNode::FilterNode() : Node("Filter")
//================================================
{
    RCLCPP_INFO(this->get_logger(), "Filter Node Init");
    
    // Get parameters
    try {
    //Declare parameters
    this->declare_parameter("laser_topic1", "Laser_Front/laser_scan");
    this->declare_parameter("laser_topic2", "Laser_Back/laser_scan");
    this->declare_parameter("frequency", 1.0);
    this->declare_parameter("angle_min", -3.14);
    this->declare_parameter("angle_max", 3.14);
    //Get Parameters from ROS2
    this->parameters.laser_topic1 = this->get_parameter("laser_topic1");
    this->parameters.laser_topic2 = this->get_parameter("laser_topic2");
    this->parameters.frequency = this->get_parameter("frequency");
    this->parameters.angle_min = this->get_parameter("angle_min");
    this->parameters.angle_max = this->get_parameter("angle_max");

    }
    catch (std::runtime_error & ex){
        RCLCPP_ERROR(this->get_logger(),"Error init parameters");
    }

    rclcpp_node_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

    // Create Subscriptions
    subscriber_laser1.subscribe(this, parameters.laser_topic1.get_value<std::string>());
    subscriber_laser2.subscribe(this, parameters.laser_topic2.get_value<std::string>());

    laser_sync_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(1),subscriber_laser1,subscriber_laser2);
    // laser_sync_-> setMaxIntervalDuration(rclcpp::Duration(parameters.frequency.get_value<double>(),0));
    laser_sync_-> registerCallback(std::bind(&FilterNode::LaserSyncCallback, 
                                                this,
                                                std::placeholders::_1,
                                                std::placeholders::_2));

    // Create Publisher
    laser_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("Laser_Merged/laser_scan", 10);
    // point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_test", 10);


    // Initialize transform listener
    tf_buffer_laser_front = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
    auto timer_interface1 = std::make_shared<tf2_ros::CreateTimerROS>(
        rclcpp_node_->get_node_base_interface(),
        rclcpp_node_->get_node_timers_interface());
    tf_buffer_laser_front->setCreateTimerInterface(timer_interface1);
    tf_listener_laser_front = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_laser_front);

    // Initialize transform listener
    tf_buffer_laser_back = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
    auto timer_interface2 = std::make_shared<tf2_ros::CreateTimerROS>(
        rclcpp_node_->get_node_base_interface(),
        rclcpp_node_->get_node_timers_interface());
    tf_buffer_laser_back->setCreateTimerInterface(timer_interface2);
    tf_listener_laser_back = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_laser_back);

}

//================================================
FilterNode::~FilterNode()
//================================================
{
}

//================================================
void FilterNode::LaserSyncCallback(
      const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg_1,
      const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg_2) 
//================================================
{
    RCLCPP_DEBUG(this->get_logger(),
                "I heard and synchronized the following timestamps: %u, %u",
                msg_1->header.stamp.sec, msg_2->header.stamp.sec);

    geometry_msgs::msg::TransformStamped tx_laser_front;
    geometry_msgs::msg::TransformStamped tx_laser_back;
    std::vector<pcl::PCLPointCloud2> clouds;
    pcl::PCLPointCloud2 tempCloud1;
    pcl::PCLPointCloud2 tempCloud2;

    // Obtenemos las transformadas del laser front a base link i del laser back a base link

    try {
        // Check if the transform is available
        tx_laser_front = tf_buffer_laser_front->lookupTransform("base_link", msg_1->header.frame_id, msg_1->header.stamp);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),"%s",ex.what());
    }
    try {
        // Check if the transform is available
        tx_laser_back = tf_buffer_laser_back->lookupTransform("base_link", msg_2->header.frame_id, msg_2->header.stamp);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),"%s",ex.what());
    }

    // Merge Laser Scan data

    // Paso 1 transformar la data de cada laser a point cloud
    // transformLaserScanToPointCloud() de la clase laser_geometry::LaserProjection
    // Paso 2 transformar los point cloude del tf del laser a base_link (puede que el laser ya lo este?? verificar)
    // tfListener_.transformPointCloud() de la clase tf::TransformListener
    // Paso 3 transformar del msg pointcloud (antiguo) a pointcloud 2
    // sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3) de la clase sensor_msgs/point_cloud_conversion.hpp
    // Paso 4 fusionar los dos pointcloud2 msgs pcl::concatenatePointCloud
    // Paso 5 transformar de pointcloud2 a laser_scan

    sensor_msgs::msg::PointCloud2 tmpCloud1Front,tmpCloud2Front;
    sensor_msgs::msg::PointCloud2 tmpCloud1Back,tmpCloud2Back;
    sensor_msgs::msg::PointCloud2 mergedCloud;

    projector_front_.transformLaserScanToPointCloud(msg_1->header.frame_id, 
                                             *msg_1, 
                                             tmpCloud1Front, 
                                             *tf_buffer_laser_front,
                                             -1, 
                                             (laser_geometry::channel_option::Distance | laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index));
    tf2::doTransform(tmpCloud1Front, tmpCloud2Front, tx_laser_front);
    pcl_conversions::toPCL(tmpCloud2Front, tempCloud1);

    projector_back_.transformLaserScanToPointCloud(msg_2->header.frame_id, 
                                             *msg_2, 
                                             tmpCloud1Back, 
                                             *tf_buffer_laser_back, 
                                             -1,
                                             (laser_geometry::channel_option::Distance | laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index));
    tf2::doTransform(tmpCloud1Back, tmpCloud2Back, tx_laser_back);
    pcl_conversions::toPCL(tmpCloud2Back, tempCloud2);
    clouds.push_back(tempCloud1);
    clouds.push_back(tempCloud2);

    // Merge
    sensor_msgs::msg::PointCloud2 merged_cloud;
    pcl::concatenatePointCloud(tmpCloud2Front, tmpCloud2Back, merged_cloud);

    // PointCloud2 to LaserScan 
    sensor_msgs::msg::LaserScan output;
	output.header.frame_id = "base_link";
	output.header.stamp = msg_1->header.stamp; 
	output.angle_min = parameters.angle_min.get_value<double>();
	output.angle_max = parameters.angle_max.get_value<double>();
	output.angle_increment = msg_1->angle_increment;
	output.time_increment = msg_1->time_increment;
	output.scan_time = msg_1->scan_time;
	output.range_min = msg_1->range_min;
	output.range_max = msg_1->range_max;

    PointCloudToLaserScan(merged_cloud, output);

    laser_scan_pub->publish(output);
}

//================================================
void FilterNode::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud, const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_info)
//================================================
{
    sensor_msgs::msg::LaserScan output;
	output.header = pcl_conversions::fromPCL(merged_cloud->header);
	output.header.frame_id = "base_link";
	output.header.stamp = laser_info->header.stamp; 
	output.angle_min = parameters.angle_min.get_value<double>();
	output.angle_max = parameters.angle_max.get_value<double>();
	output.angle_increment = laser_info->angle_increment;
	output.time_increment = laser_info->time_increment;
	output.scan_time = laser_info->scan_time;
	output.range_min = laser_info->range_min;
	output.range_max = laser_info->range_max;

	uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
	output.ranges.assign(ranges_size, output.range_max + 1.0);

    int intensities_idx = getFieldIndex(*merged_cloud, "intensities");
    int intensities_offset;
    float intensitis;
    if (intensities_idx != -1)
    {
        //intensities present
        output.intensities.resize(ranges_size);
        intensities_offset = merged_cloud->fields[intensities_idx].offset;
    }
	for(int i=0; i<points.cols(); i++)
	{
		const float &x = points(0,i);
		const float &y = points(1,i);
		const float &z = points(2,i);
        if (intensities_idx != -1)
        {
            //intensities present
            memcpy (&intensitis, &merged_cloud->data[intensities_offset], sizeof (float));
            intensities_offset += merged_cloud->point_step;
        }

		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			RCLCPP_DEBUG(this->get_logger(),"rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		double range_sq = y*y+x*x;
		double range_min_sq_ = output.range_min * output.range_min;
		if (range_sq < range_min_sq_) {
			RCLCPP_DEBUG(this->get_logger(),"rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output.angle_min || angle > output.angle_max)
		{
			RCLCPP_DEBUG(this->get_logger(),"rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
			continue;
		}
		int index = (angle - output.angle_min) / output.angle_increment;


		if (output.ranges[index] * output.ranges[index] > range_sq)
        {
            output.ranges[index] = sqrt(range_sq);
            if (intensities_idx != -1)
            {
                //intensities present
                output.intensities[index] = intensitis;
            }
        }
	}

	laser_scan_pub->publish(output);
}

/** Modified function from package::pointcloud_to_laserscan: pointcloud_to_laserscan_node.cpp **/
void FilterNode::PointCloudToLaserScan(
    sensor_msgs::msg::PointCloud2 & cloud_msg,
    sensor_msgs::msg::LaserScan & scan_msg)
{
    // determine amount of rays to create
    uint32_t ranges_size = std::ceil(
      (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    scan_msg.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x"),
      iter_y(cloud_msg, "y"), iter_z(cloud_msg, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
            RCLCPP_DEBUG(
            this->get_logger(),
            "rejected for nan in point(%f, %f, %f)\n",
            *iter_x, *iter_y, *iter_z);
            continue;
        }

        double range = hypot(*iter_x, *iter_y);
        if (range < scan_msg.range_min) {
            RCLCPP_DEBUG(
            this->get_logger(),
            "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
            range, scan_msg.range_min, *iter_x, *iter_y, *iter_z);
            continue;
        }
        if (range > scan_msg.range_max) {
            RCLCPP_DEBUG(
              this->get_logger(),
              "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
              range,  scan_msg.range_max, *iter_x, *iter_y, *iter_z);
            continue;
        }

        double angle = atan2(*iter_y, *iter_x);
        if (angle < scan_msg.angle_min || angle > scan_msg.angle_max) {
            RCLCPP_DEBUG(
              this->get_logger(),
              "rejected for angle %f not in range (%f, %f)\n",
              angle, scan_msg.angle_min, scan_msg.angle_max);
            continue;
        } 

        // overwrite range at laserscan ray if new range is smaller
        int index = (angle - scan_msg.angle_min) / scan_msg.angle_increment;
        if (range < scan_msg.ranges[index]) {
            scan_msg.ranges[index] = range;
        }
    }
}   

//================================================
rcl_interfaces::msg::SetParametersResult FilterNode::dyn_reconf_callback(const std::vector<rclcpp::Parameter> & parameters)
//================================================
{
    RCLCPP_INFO(this->get_logger(), "Parameter change request");
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "The reason could not be allowed";
    for (const auto & parameter : parameters){
        try {
            if (parameter.get_name() == "frequency" && parameter.get_value<double>() != this->parameters.frequency.get_value<double>()){
                result.successful = true;
                result.reason = "Parameter change accepted";
                RCLCPP_INFO(this->get_logger(), "Parameter change accepted '%s'", parameter.get_name().c_str());
                this->parameters.frequency = parameter;
                RCLCPP_INFO(this->get_logger(),"Parameter '%s' changed",parameter.get_name().c_str());
            }
            // else if (parameter.get_name() == "laser_topic2" && parameter.get_value<std::string>() != this->parameters.local_ip.get_value<std::string>()){
            //     result.successful = true;
            //     result.reason = "Parameter change accepted";
            //     RCLCPP_INFO(this->get_logger(), "Parameter change accepted",parameter.get_name().c_str());
            //     this->parameters.local_ip = parameter;
            //     RCLCPP_INFO(this->get_logger(),"Parameter '%s' changed",parameter.get_name().c_str());
            // }         
            else{
                throw std::invalid_argument("Parameter not found or not available to reconfigure");
            }
            // Volvemos a etapa StandBy
            // ToggleState(StandBy_timer);
        }
        catch (const std::invalid_argument& e){
            RCLCPP_WARN(this->get_logger(),"Error changing parameter: '%s'",e.what());
        }
        catch(std::exception & e){
            RCLCPP_WARN(this->get_logger(),e.what());
            result.reason = e.what();
            RCLCPP_INFO(this->get_logger(),"Parameter change cancelled '%s'", parameter.get_name().c_str());
        }
        catch (...){
            RCLCPP_WARN(this->get_logger(),"Parameter change cancelled '%s'", parameter.get_name().c_str());
        }
    }
    return result;
}

} // namespace atlas_filters
