/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Carles Vela, Martí Bolet, María Mercadé
   Contact: support.idi@movvo.eu
*/

#include <thread>
#include <chrono>         // std::chrono::seconds

#include "atlas_imu/imu.hpp"
#include <stdexcept>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using namespace atlas_utils::sm;

namespace atlas_imu {

// Constructor
//================================================
ImuNode::ImuNode() : Node("IMU"),
					StateMachine(std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {}))
//================================================
{
	// Define the State Machine timers at the desired frequency and link them to their callbacks
    Unconfigured_timer = this->create_wall_timer(500ms, std::bind(&ImuNode::Unconfigured, this));
    StandBy_timer = this->create_wall_timer(500ms, std::bind(&ImuNode::StandBy, this));
    Run_timer = this->create_wall_timer(8ms, std::bind(&ImuNode::Run, this));
    Fault_timer = this->create_wall_timer(2s, std::bind(&ImuNode::Fault, this));
    ShutDown_timer = this->create_wall_timer(500ms, std::bind(&ImuNode::ShutDown, this));
    
    // Cancel all timers except the initial one
    StandBy_timer->cancel();
    Run_timer->cancel();
    Fault_timer->cancel();
    ShutDown_timer->cancel();

    // Initialize parameters_
    InitializeImu();

    nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    sm_ = std::shared_ptr<::atlas_utils::sm::StateMachine>(this, [](::atlas_utils::sm::StateMachine *) {});
}

//================================================
ImuNode::~ImuNode()
//================================================
{
}

/*****************************************************************************
                    State Machine: FUNCIONES ESTADO NODO
*****************************************************************************/

//================================================
void ImuNode::Unconfigured()
//================================================
{
    // Update actual state to UNCONFIGURED
    updateState(UNCONFIGURED);

    RCLCPP_DEBUG(this->get_logger(),"On Unconfigured");

    // Try to configure
    if (ConfigureImu()){
        // If configured sucesfully go to STANDBY
        ToggleState(StandBy_timer);
    }
    else{
        RCLCPP_DEBUG(this->get_logger(),"Configuration failed");
    }
    // Check if there is an error
    if (onFault){
        // If there is an error go to FAULT
        ToggleState(Fault_timer);
    }
}

// Logic to apply when node is in normal state but not working
//================================================
void ImuNode::StandBy()
//================================================
{
    // Update actual state to STANDBY
    updateState(STANDBY);
    RCLCPP_DEBUG(this->get_logger(),"On Stand By");

    if (parameters_.autorun.as_bool()){
        // Reset autorun to false
        this->set_parameter(rclcpp::Parameter("autorun",false));

        ToggleState(Run_timer);
    }

    // Check if there is an error
    if (onFault){
        ToggleState(Fault_timer);
    }
}

// Logic to apply when node is in normal state and running
//================================================
void ImuNode::Run()
//================================================
{
    // Update actual state to RUN
    updateState(RUN);
    RCLCPP_DEBUG(this->get_logger(),"On Run");

	// Publish Imu info
    PublishImuInfo();

    // Check if there is an error
    if (onFault) {
        ToggleState(Fault_timer);
    }
}

// Logic to apply when node has critical errors	
//================================================
void ImuNode::Fault()
//================================================
{
    // Update actual state to FAULT
    updateState(FAULT);
    RCLCPP_DEBUG(this->get_logger(),"On Fault");
	
    // Check error by error and try to correct them if possible
    if (driver_->ErrorHandle()){
        ToggleState(Run_timer);
    }
    else if (critical & IMU2USE_DOESNT_EXIST){
        RCLCPP_ERROR(this->get_logger(), "IMU Device to use does not exist or is not configured. Can't solve error: going to shutdown");
        ToggleState(ShutDown_timer);
    }
    
    if (sm_->critical == 0){
        RCLCPP_INFO(nh_->get_logger(), "All errors solved, going to run");
        ToggleState(Run_timer);
    }
}

//================================================
void ImuNode::ShutDown()
//================================================
{
    // Update actual state to SHUTDOWN
    updateState(SHUTDOWN);

    // Disconnect
	Release();
}


/*****************************************************************************
                FUNCIONES PROPIAS FUNCIONAMIENTO NODO
*****************************************************************************/
// Declare parameters_
//================================================
void ImuNode::InitializeImu() 
//================================================
{
    this->declare_parameter("autorun", true);
    this->declare_parameter("imu_topic", "imu");
}

// Method to set and configure driver
//================================================
void ImuNode::SetDriver(std::shared_ptr<imu_drivers::DriverAbstract> driver)
//================================================
{
    // Set driver and configure it
    driver_ = driver;
    driver_->SetNode(nh_);
    driver_->setSM(sm_);
    driver_->Initialize();
    driver_->Configure();
}

// Initialize the Odometry Node
bool ImuNode::ConfigureImu() {
	// Retrieving parameters of this node
	try {
        // Get parameters
		parameters_.autorun    = this->get_parameter("autorun");
        parameters_.imu_topic    = this->get_parameter("imu_topic");

        // Test type
        parameters_.autorun.as_bool();
        parameters_.imu_topic.as_string();

        auto null = this->add_on_set_parameters_callback(std::bind(&ImuNode::dyn_reconf_callback, this, _1));
    } catch (...) {
		RCLCPP_ERROR(this->get_logger(),"Error getting parameters of IMU node");
        return false;
	}

    try{
        // Initialize publisher
        std::string imu_topic = this->get_name() + std::string("/") + parameters_.imu_topic.as_string();
	    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
    }
    catch(...){
        RCLCPP_ERROR(this->get_logger(),"imu publisher NOT created");
        return false;
    }
    
    if (driver_ == nullptr){
        RCLCPP_ERROR(this->get_logger(),"Selected IMU device doesn't exist or is not created");
        updateErrorCritical(IMU2USE_DOESNT_EXIST, IMU2USE_DOESNT_EXIST_DESC);
        return false;
    }

    // Connect to driver
    if (!ConnectImu()) {
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Imu node configured sucessfully");
	
	return true;
}

// Connect to Driver
//================================================
bool ImuNode::ConnectImu() 
//================================================
{
    try  {
        bool con = driver_->Connect();
        if (!con) {
            updateErrorCritical(NOT_CONNECTED, NOT_CONNECTED_DESC);
            return false;
        } else {
            updateSolvedCritical(NOT_CONNECTED, NOT_CONNECTED_DESC);
            return true;
        }
    }
    catch (...) {
        updateErrorCritical(NOT_CONNECTED, NOT_CONNECTED_DESC);
        return false;
    }
    return true;
}

// Disconnect from driver
//================================================
void ImuNode::Release()
//================================================
{
    rclcpp::shutdown();
}

// Publish IMU info
//================================================
void ImuNode::PublishImuInfo()
//================================================
{
    sensor_msgs::msg::Imu msg = driver_->GetImuMsg();
    imu_pub_->publish(msg);
}

// Dynamic reconfigure
//================================================
rcl_interfaces::msg::SetParametersResult ImuNode::dyn_reconf_callback(const std::vector<rclcpp::Parameter> & parameters)
//================================================
{
    RCLCPP_INFO(this->get_logger(), "Parameter change request");
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "Parameter change accepted";
    for (const auto & parameter : parameters){
        try{
            if (parameter.get_name() == "autorun" && 
                parameter.as_bool() != parameters_.autorun.as_bool())   {
                parameters_.autorun = parameter;
            }
			else if (parameter.get_name() == "imu_topic" && 
                parameter.as_string() != parameters_.imu_topic.as_string())    {
                parameters_.imu_topic = parameter;
            }
            // Parameters of driver
            else if (boost::algorithm::contains(parameter.get_name(), driver_->GetName()))  {
                driver_->Configure();
            }
            else{
				RCLCPP_ERROR(this->get_logger(), "Parameter not found or not available to reconfigure");
                throw std::invalid_argument("Parameter not found or not available to reconfigure");
            }
        }
        catch(std::exception & e){
            RCLCPP_WARN(this->get_logger(),e.what());
            result.successful = false;
            result.reason = e.what();
            RCLCPP_INFO(this->get_logger(),"Parameter change cancelled %s",parameter.get_name().c_str());
            continue;
        }
        catch (...){
            result.successful = false;
            result.reason = "Reason unknown";
            RCLCPP_WARN(this->get_logger(),"Parameter change cancelled %s", parameter.get_name().c_str());
            continue;
        }

        // Success
        RCLCPP_INFO(this->get_logger(),"Parameter '%s' changed", parameter.get_name().c_str());
    }
    return result;
}


} // namespace atlas_imu