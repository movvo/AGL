#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <math.h>
#include "ageve_interfaces/msg/channel_values.hpp"

namespace roboteq {

// ref) http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#goal-velocity104
//constexpr double RPM_TO_MS = 0.229 * 0.0034557519189487725;

// 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
//constexpr double TICK_TO_RAD = 0.001533981;

class Odometry {
  public:
	Odometry(std::shared_ptr<rclcpp::Node> &nh);
	void spin();

  private:
	// Config parameters
	std::shared_ptr<rclcpp::Node> nh_;
	typedef struct {
		double wheel_separation;
		double wheel_radius;
		std::string frame_id;
		std::string child_frame_id;
		std::string js_topic;
		bool publish_tf;
		bool publish_odom;
		bool publish_js;
  	} OdomParams;
	OdomParams params;

	// Subscribers and publishers
	rclcpp::Subscription<ageve_interfaces::msg::ChannelValues>::SharedPtr l_wheel_sub;
	rclcpp::Subscription<ageve_interfaces::msg::ChannelValues>::SharedPtr r_wheel_sub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;

	std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

	//Encoder related parameters
	double encoder_min;
	double encoder_max;
	double ticks_meter;
	double base_width;
	double rate;

	double encoder_low_wrap;
	double encoder_high_wrap;
	double enc_left;
	double enc_right;
	
	double prev_lencoder;
	double prev_rencoder;

	double lmult;
	double rmult;

	double left;
	double right;

	// ROS2 Timers
	rclcpp::Time t_next;
	rclcpp::Time then;
	rclcpp::Time current_time, last_time;

	double dx;
	double dr;
	double x_final, y_final, theta_final;

	// Functions
	void Initialize();
	void leftencoderCb(const ageve_interfaces::msg::ChannelValues::SharedPtr left_ticks);
	void rightencoderCb(const ageve_interfaces::msg::ChannelValues::SharedPtr right_ticks);
	void update();
};
} // namespace
