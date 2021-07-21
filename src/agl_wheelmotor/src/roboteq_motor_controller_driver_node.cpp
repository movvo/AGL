#include <me00_wheelmotor/roboteq_motor_controller_driver_node.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace roboteq;

// Constructors
Driver_Node::Driver_Node() : Node("driver_node") {
	internal_state.state = ageve_interfaces::msg::Diagnostics::UNCONFIGURED;
	if(Initialize()) {
		Run();
	}
}

Driver_Node::~Driver_Node(){
}

// Initialize the Driver_Node parameters
bool Driver_Node::Initialize() {
	internal_state.state = ageve_interfaces::msg::Diagnostics::UNCONFIGURED;
	// Declare and get parameters
	this->declare_parameter<int>("frequencyH", 50);
	this->declare_parameter<int>("frequencyL", 100);
	this->declare_parameter<int>("frequencyG", 100);
	// queryH
	this->declare_parameter<std::string>("queryH.motor_amps", "?A");
	this->declare_parameter<std::string>("queryH.motor_command", "?M");
	this->declare_parameter<std::string>("queryH.encoder_count", "?C");
	this->declare_parameter<std::string>("queryH.encoder_speed", "?S");
	// queryL
	this->declare_parameter<std::string>("queryL.volts", "?V");
	this->declare_parameter<std::string>("queryL.feedback", "?F");
	this->declare_parameter<std::string>("queryL.battery_amps", "?BA");
	this->declare_parameter<std::string>("queryL.power", "?P");
	// queryG
	this->declare_parameter<std::string>("queryG.fault_flag", "?FF");

	// Get parameters
	this->get_parameter("use_odom", use_odom);
	this->get_parameter("frequencyH", frequencyH);
	this->get_parameter("frequencyL", frequencyL);
	this->get_parameter("frequencyG", frequencyG);

	// Init diagnostics variables
	diagnostics_publisher = this->create_publisher<ageve_interfaces::msg::Diagnostics>("status", 10);
	timer_ = this->create_wall_timer(100ms, std::bind(&Driver_Node::diagnostics_callback, this));
	internal_state.anybus_error = 0;
	internal_state.controller_error  = 0;
	internal_state.motor_right_error = 0;
	internal_state.motor_left_error  = 0;

	// Init external classes
	nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
	if(use_odom) odom_ = std::make_unique<Odometry>(nh_);
    controller_ = std::make_unique<Controller>(nh_);
	// controller_->Connect();

	// Create controller class and try to connect 1 time
    controller_eip_ = std::make_unique<roboteq::Controller_EIP>(nh_);
	connect_timmer_ = this->create_wall_timer(3s, std::bind(&Driver_Node::connect_callback, this));
	controller_eip_->Connect();

	// Check Anybus conection
	// EIP_INT anybus_status = controller_eip_->get(CONTROL_STATUS_WORD, 0x00);
	// bool anybus_init = anybus_status&INIT>>1;
	// bool anybus_runs = anybus_status&RUN >>1;

	// Queries of H
	std::vector<std::string> KeysH_vector = {
		"motor_amps",
		"motor_command",
		"encoder_count",
		"encoder_speed"
	};
	std::stringstream ss_h;
	ss_h << "# c_/\"DH?\",\"?\"";
	for(int i=0; i<KeysH_vector.size(); i++){
		// Concatenate in the string parameter settings
		ss_h << this->get_parameter("queryH." + KeysH_vector[i]).value_to_string() << "_";
		// Generate publisher for this parameter
		publisherVecH.emplace_back(this->create_publisher<ageve_interfaces::msg::ChannelValues>(KeysH_vector[i], 100));
	}
	ss_h << "# " << frequencyH << "_";

	// Queries of L
	std::vector<std::string> KeysL_vector = {
		"volts",
		"feedback",
		"battery_amps",
		"power"
	};
	std::stringstream ss_l;
	ss_l << "/\"DL?\",\"?\"";
	for(int i=0; i<KeysL_vector.size(); i++){
		// Concatenate in the string parameter settings
		ss_l << this->get_parameter("queryL." + KeysL_vector[i]).value_to_string() << "_";
		// Generate publisher for this parameter
		publisherVecL.emplace_back(this->create_publisher<ageve_interfaces::msg::ChannelValues>(KeysL_vector[i], 100));
	}
	ss_l << "# " << frequencyL << "_";

	// Queries of G
	std::vector<std::string> KeysG_vector = {
		"fault_flag"
	};
	std::stringstream ss_g;
	ss_g << "/\"DG?\",\"?\"";
	for(int i=0; i<KeysG_vector.size(); i++){
		// Concatenate in the string parameter settings
		ss_g << this->get_parameter("queryG." + KeysG_vector[i]).value_to_string() << "_";
		// Generate publisher for this parameter
		publisherVecG.emplace_back(this->create_publisher<std_msgs::msg::String>(KeysG_vector[i], 100));
	}
	ss_g << "# " << frequencyG << "_";

	// Send the Config String via ethernet to driver
	// controller_->Send("^echof 1_");
	// controller_->Send(ss_h.str());
	// controller_->Send(ss_l.str());
	// controller_->Send(ss_g.str());

	// Init publisher and services callbacks
	read_publisher = this->create_publisher<std_msgs::msg::String>("read", 1000);

	cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("Control/cmd_vel", 10, std::bind(&Driver_Node::cmd_vel_callback, this, _1));

	configsrv = this->create_service<ageve_interfaces::srv::Config>("config_service",
							std::bind(&Driver_Node::configservice, this, _1, _2));
	commandsrv = this->create_service<ageve_interfaces::srv::Command>("command_service",
							std::bind(&Driver_Node::commandservice, this, _1, _2));
	maintenancesrv = this->create_service<ageve_interfaces::srv::Maintenance>("maintenance_service",
							std::bind(&Driver_Node::maintenanceservice, this, _1, _2));

	internal_state.state = ageve_interfaces::msg::Diagnostics::STANDBY;
	RCLCPP_INFO(this->get_logger(), "Roboteq Driver Initialized succesfully");
}

// CALLBACKS FUNCTIONS
// Callback function of commands: Twist msgs
void Driver_Node::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
	if (!controller_eip_->Available()) {
		return;
	}
	std::stringstream cmd_sub;
	cmd_sub << "!G " << MOTOR_LEFT << " " << msg->linear.x << "_"
			<< "!G " << MOTOR_LEFT << " " << msg->angular.z << "_";

	// Write the string command to ethernet client
	controller_->Send(cmd_sub.str());

	// Write the command via Anybus
	controller_eip_->set(CANGO, MOTOR_LEFT,  (EIP_INT) msg->linear.x );  // TODO: Transform value to the desired
	controller_eip_->set(CANGO, MOTOR_RIGHT, (EIP_INT) msg->angular.z);  // TODO: Transform value to the desired

	RCLCPP_INFO_STREAM(this->get_logger(), cmd_sub.str());
}

// Callback of timers
void Driver_Node::connect_callback() {
	// If controller is not connected, retry
	if (!controller_eip_->Available()) {
		controller_eip_->Connect();
	}
}

void Driver_Node::diagnostics_callback() {
	// Refresh the diagnostics_msg
	diagnostics_msg.header.stamp = this->now();
    // diagnostics_msg.connected = controller_->Available(); // Old Controller
	// diagnostics_msg.connected = controller_eip_->Available(); // New EIP Controller
    // diagnostics_msg.error.warning  = 
	diagnostics_msg.error.critical |= internal_state.anybus_error << ANYBUS_ERROR |
									  internal_state.controller_error << CONTROLLER_ERROR |
									  internal_state.motor_right_error << MOTOR_RIGHT_ERROR |
									  internal_state.motor_left_error  << MOTOR_LEFT_ERROR;
    diagnostics_msg.state = internal_state.state;
	if (diagnostics_msg.error.critical > 0) {
		diagnostics_msg.state = ageve_interfaces::msg::Diagnostics::FAULT;
	}
	diagnostics_publisher->publish(diagnostics_msg);
}

// SERVICES METHODS
void Driver_Node::configservice(const std::shared_ptr<ageve_interfaces::srv::Config::Request> req, std::shared_ptr<ageve_interfaces::srv::Config::Response> res) {
	std::stringstream str;
	str << "^" << req->user_input << " " << req->channel << " " << req->value << "_ "
		<< "%\clsav321654987";
	controller_->Send(str.str());
	res->result = str.str();

	RCLCPP_INFO_STREAM(this->get_logger(), res->result);
}

void Driver_Node::commandservice(const std::shared_ptr<ageve_interfaces::srv::Command::Request> req, std::shared_ptr<ageve_interfaces::srv::Command::Response> res) {
	std::stringstream str;
	str << "!" << req->user_input << " " << req->channel << " " << req->value << "_";
	controller_->Send(str.str());
	res->result = str.str();

	RCLCPP_INFO_STREAM(this->get_logger(), res->result);
}

void Driver_Node::maintenanceservice(const std::shared_ptr<ageve_interfaces::srv::Maintenance::Request> req, std::shared_ptr<ageve_interfaces::srv::Maintenance::Response> res) {
	std::stringstream str;
	str << "%" << req->user_input << " " << "_";
	res->result = controller_->Send(str.str());

	RCLCPP_INFO_STREAM(this->get_logger(), res->result);
}


// RUN METHOD
void Driver_Node::Run() {
	rclcpp::Rate loop_rate(5);
	while (rclcpp::ok()){
		std_msgs::msg::String result;
		try {
			// Check for connection
			if (!controller_eip_->Available()) {
				return;
			}
			// Read the data string from ethernet client
			controller_->Read(&result.data);

			// Read the status of Anybus
			// Mask the 4 bits of General Counter Error and convert it to uint8_t 
			internal_state.anybus_error = controller_eip_->get(CONTROL_STATUS_WORD, 0x00)&GEC>>8; 

			// Read the data via Anybus
			EIP_INT motor_amps_left  = controller_eip_->get(MOTOR_AMPS, MOTOR_LEFT);
			EIP_INT motor_amps_right = controller_eip_->get(MOTOR_AMPS, MOTOR_RIGHT);

			EIP_INT encoder_count_left  = controller_eip_->get(ENCODER_COUNT, MOTOR_LEFT);
			EIP_INT endocer_count_right = controller_eip_->get(ENCODER_COUNT, MOTOR_RIGHT);

			internal_state.controller_error = controller_eip_->get(FAULT_FLAG, 0x00);

			internal_state.motor_left_error  = controller_eip_->get(FAULT_MOTOR, MOTOR_LEFT);
			internal_state.motor_right_error = controller_eip_->get(FAULT_MOTOR, MOTOR_RIGHT);
		} catch (...) {
			internal_state.state = ageve_interfaces::msg::Diagnostics::FAULT;
		}

		read_publisher->publish(result);
		boost::replace_all(result.data, "\r", "");
		boost::replace_all(result.data, "+", "");

		std::vector<std::string> fields;
		std::vector<std::string> fields_H;
		std::vector<std::string> fields_L;
		std::vector<std::string> fields_G;
		
		boost::split(fields, result.data, boost::algorithm::is_any_of("D"));
		boost::split(fields_H, fields[1], boost::algorithm::is_any_of("?"));
		boost::split(fields_L, fields[2], boost::algorithm::is_any_of("?"));
		boost::split(fields_G, fields[3], boost::algorithm::is_any_of("?"));

		if (fields_H[0] == "H") {
			for (int i = 0; i < publisherVecH.size(); ++i) {
				std::vector<std::string> sub_fields_H;

				boost::split(sub_fields_H, fields_H[i + 1], boost::algorithm::is_any_of(":"));
				ageve_interfaces::msg::ChannelValues Q1;
				
				Q1.value.push_back(0);
				for (int j = 0; j < sub_fields_H.size(); j++) {
					Q1.value.push_back(boost::lexical_cast<int>(sub_fields_H[j]));
				}

				publisherVecH[i]->publish(Q1);
			}
		}
		else {
			RCLCPP_INFO_STREAM(this->get_logger(), "Garbage data on Serial");
		}

		if (fields_L[0] == "L") {
			for (int i = 0; i < publisherVecL.size(); ++i) {
				std::vector<std::string> sub_fields_L;

				boost::split(sub_fields_L, fields_L[i + 1], boost::algorithm::is_any_of(":"));

				ageve_interfaces::msg::ChannelValues Q1;
				Q1.value.push_back(0);
				for (int j = 0; j < sub_fields_L.size(); j++) {
					Q1.value.push_back(boost::lexical_cast<int>(sub_fields_L[j]));
				}

				publisherVecL[i]->publish(Q1);
			}
		}

		if (fields_G[0] == "G") {
			for (int i = 0; i < publisherVecG.size(); ++i) {
				std_msgs::msg::String Q1;
				Q1.data = fields_G[i + 1];

				publisherVecG[i]->publish(Q1);
			}
		}

		RCLCPP_INFO_STREAM(this->get_logger(), "success!");

		loop_rate.sleep();
	}
}