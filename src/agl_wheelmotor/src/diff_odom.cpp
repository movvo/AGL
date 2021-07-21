/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Iñaki Lorente
   Contact: support.idi@ageve.net
*/

#include "agl_wheelmotor/diff_odom.hpp"

using std::placeholders::_1;

Odometry::Odometry(std::shared_ptr<rclcpp::Node> &nh) : nh_(nh) {
	Initialize();
}

void Odometry::Initialize() {
	//Retrieving parameters of nh_ node
	nh_->declare_parameter<double>("odom.wheel_separation", 0.2);
    nh_->declare_parameter<double>("odom.wheel_radius", 0.1);
    nh_->declare_parameter<std::string>("odom.frame_id", "odom");
    nh_->declare_parameter<std::string>("odom.child_frame_id", "base_link");
	nh_->declare_parameter<bool>("odom.publish_tf", false);
	nh_->declare_parameter<bool>("odom.publish_odom", true);
	nh_->declare_parameter<bool>("odom.publish_js", true);
	nh_->declare_parameter<std::string>("odom.js_topic", "joint_states");

    // Get parameters
    nh_->get_parameter("odom.wheel_separation", params.wheel_separation);
    nh_->get_parameter("odom.wheel_radius", params.wheel_radius);
	nh_->get_parameter("odom.frame_id", params.frame_id);
    nh_->get_parameter("odom.child_frame_id", params.child_frame_id);
	nh_->get_parameter("odom.publish_tf", params.publish_tf);
	nh_->get_parameter("odom.publish_js", params.publish_js);
	nh_->get_parameter("odom.publish_odom", params.publish_odom);
	nh_->get_parameter("odom.js_topic", params.js_topic);

	RCLCPP_INFO(nh_->get_logger(), "Started odometry computing node");
	l_wheel_sub = nh_->create_subscription<ageve_interfaces::msg::ChannelValues>("encoder_count", 1000,
	 				std::bind(&Odometry::leftencoderCb, this, _1));  // TODO: Check topic names
	r_wheel_sub = nh_->create_subscription<ageve_interfaces::msg::ChannelValues>("encoder_count", 1000,
					std::bind(&Odometry::rightencoderCb, this, _1));

  	odom_pub = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
	odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh_->shared_from_this());

	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
	js_pub = nh_->create_publisher<sensor_msgs::msg::JointState>(params.js_topic, qos);

	prev_lencoder = 0;
	prev_rencoder = 0;

	lmult = 0;
	rmult = 0;

	left  = 0;
	right = 0;

	encoder_min = -65536;
	encoder_max = 65536;

	rate = 10;
	ticks_meter = 50;

	base_width = 0.3;

	encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
	encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;

	t_next = nh_->now() + rclcpp::Duration(0, (int)1/rate*1e9);
	then = nh_->now();

	enc_left = 10;
	enc_right = 0;

	dx = 0;
	dr = 0;
 
	x_final = 0;
	y_final = 0;
	theta_final = 0;
	
	current_time = nh_->now();
  	last_time = nh_->now();
}

// Spin function
void Odometry::spin(){
     rclcpp::Rate loop_rate(rate);

     while (rclcpp::ok()){
		update();
		loop_rate.sleep();
	}
}

// Update function
void Odometry::update(){
	rclcpp::Time now = nh_->now();

	double elapsed;
	double d_left, d_right, d, th,x,y;

	if ( now > t_next) {
		elapsed = now.seconds() - then.seconds(); 

		if(enc_left == 0){
			d_left = 0;
			d_right = 0;
		} else {
			d_left  = (left-enc_left) / (ticks_meter);
			d_right = (right-enc_right) / (ticks_meter);
		}
		
		enc_left = left;
		enc_right = right;

		d = (d_left+d_right) / 2.0;

		RCLCPP_INFO_STREAM(nh_->get_logger(), d_left << " : " << d_right);

		th = (d_right-d_left) / base_width;
		dx = d /elapsed;
		dr = th / elapsed;
	
		if ( d != 0){
			x = cos(th) * d;
			y = -sin(th) * d;
			// calculate the final position of the robot
			x_final = x_final + ( cos( theta_final ) * x - sin( theta_final ) * y );
			y_final = y_final + ( sin( theta_final ) * x + cos( theta_final ) * y );
		}

		if( th != 0)
			theta_final = theta_final + th;

		geometry_msgs::msg::Quaternion odom_quat;

		odom_quat.x = 0.0;
		odom_quat.y = 0.0;
		odom_quat.z = 0.0;

		odom_quat.z = sin(theta_final/2);	
		odom_quat.w = cos(theta_final/2);

		// First, we'll publish the transform over tf
		if (params.publish_tf) {
			geometry_msgs::msg::TransformStamped odom_trans;
			odom_trans.header.stamp = now;
			odom_trans.header.frame_id = params.frame_id;
			odom_trans.child_frame_id = params.child_frame_id;

			odom_trans.transform.translation.x = x_final;
			odom_trans.transform.translation.y = y_final;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			// Send the transform
			odom_broadcaster->sendTransform(odom_trans);
		}
		// Next, we'll publish the odometry message over ROS
		nav_msgs::msg::Odometry odom;
		odom.header.stamp = now;
		odom.header.frame_id = params.frame_id;

		// Set the position
		odom.pose.pose.position.x = x_final;
		odom.pose.pose.position.y = y_final;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		// Set the velocity
		odom.child_frame_id = params.child_frame_id;
		odom.twist.twist.linear.x = dx;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = dr;

		// Publish the message
		if (params.publish_odom) {
			odom_pub->publish(odom);
		}

		// Publish the JointStates
		auto msg = std::make_unique<sensor_msgs::msg::JointState>();
		static std::array<uint32_t, 2> last_diff_position, last_position;
		
		// Get encoder values and velocity
		// TODO: Check if these are the values we want
		std::array<uint32_t,2> position = {d_right, d_left};
		//TODO: Tengo dudas si esto deberia ser (dx-th) y (dx+th)
		std::array<uint32_t,2> velocity = {dx, dx}; 
		
		// Set/Update header joint state (equal to odom)
		msg->header.frame_id = params.child_frame_id;
		msg->header.stamp = odom.header.stamp;
		
		// Joint states to use (must be equal to the ones on Description/URDF)
		msg->name.push_back("left_wheel_joint");
		msg->name.push_back("right_wheel_joint");

		// TODO: Get this values correctly. We should check this!!
		msg->position.push_back(last_diff_position[0]);
		msg->position.push_back(last_diff_position[1]);
		msg->velocity.push_back(velocity[0]);
		msg->velocity.push_back(velocity[1]);

		// Joint State current position based on previous position
		last_diff_position[0] += (position[0] - last_position[0]);
		last_diff_position[1] += (position[1] - last_position[1]);
		last_position = position;
		
		// Publish Joint States
		if (params.publish_js) {
			js_pub->publish(std::move(msg));
		}
		
		// Refresh the times
		then = now;

		RCLCPP_INFO_STREAM(nh_->get_logger(), "dx =" << x_final);
		RCLCPP_INFO_STREAM(nh_->get_logger(), "dy =" << y_final);
	}
	else {
		// ?
	}
	RCLCPP_INFO_STREAM(nh_->get_logger(), "Not in loop");		
}

void Odometry::leftencoderCb(const ageve_interfaces::msg::ChannelValues::SharedPtr left_ticks) {
	RCLCPP_INFO_STREAM(nh_->get_logger(), "Left tick" << left_ticks->value[1]);
	double enc = left_ticks->value[1];

	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))	{
		lmult = lmult + 1;
	}
	
	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))	{
		lmult = lmult - 1;
	}
	left = 1.0 * (enc + lmult * (encoder_max - encoder_min ));
	prev_lencoder = enc;
	RCLCPP_INFO_STREAM(nh_->get_logger(), "Left " << left);
}

//Right encoder callback
void Odometry::rightencoderCb(const ageve_interfaces::msg::ChannelValues::SharedPtr right_ticks) {
	RCLCPP_INFO_STREAM(nh_->get_logger(), "Right tick" << right_ticks->value[2]);
	double enc = right_ticks->value[2];
	
	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap)) {
		rmult = rmult + 1;
	}
	
	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))	{
		rmult = rmult - 1;
	}

	right = 1.0 * (enc + rmult * (encoder_max - encoder_min ));
	prev_rencoder = enc;

	RCLCPP_INFO_STREAM(nh_->get_logger(), "Right " << right);
}
