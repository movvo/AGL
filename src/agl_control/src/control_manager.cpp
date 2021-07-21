/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Martí Bolet, Albert Arlà
   Contact: support.idi@ageve.net
*/

#include "me00_control/control_manager.hpp"
#include "me00_control/errors.hpp"

// SM
using namespace ageve_utils::sm;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/*****************************************************************************
                    ControlManager: NODE CONSTRUCTOR
*****************************************************************************/

//================================================
ControlManager::ControlManager() : Node("ControlManager"),
                       StateMachine(std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {}))
//================================================
{
    // Definir los State Machine timers a la frequencia deseada
    // y vincularlos a sus callback
    Unconfigured_timer = this->create_wall_timer(500ms, std::bind(&ControlManager::Unconfigured,this));
    StandBy_timer = this->create_wall_timer(500ms, std::bind(&ControlManager::StandBy,this));
    Run_timer = this->create_wall_timer(500ms, std::bind(&ControlManager::Run,this));
    Fault_timer = this->create_wall_timer(500ms, std::bind(&ControlManager::Fault,this));
    ShutDown_timer = this->create_wall_timer(500ms, std::bind(&ControlManager::ShutDown,this));
    
    // Cancelar todos los timers excepto el inicial
    StandBy_timer->cancel();
    Run_timer->cancel();
    Fault_timer->cancel();
    ShutDown_timer->cancel();

    // Declarar parametros
    this->declare_parameter("autorun");
    this->declare_parameter("manual_pkg");
    this->declare_parameter("autonomous_pkg");
    this->declare_parameter("initial_mode");
    this->declare_parameter("scan_front");
    this->declare_parameter("scan_back");
}

/*****************************************************************************
                    State Machine: FUNCIONES ESTADO NODO
*****************************************************************************/

//================================================
void ControlManager::Unconfigured()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(UNCONFIGURED);

    // Introducir las acciones a realizar
    // y la logica de cambio de estado
    if (Initialize()) //Condicion para pasar al siguiente
    { 
        ToggleState(StandBy_timer);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"Fallo inicializacion");
        ToggleState(Fault_timer);    
    }

    // Check if there is an error
    if (onFault){
        ToggleState(Fault_timer);
    }
}

//================================================
void ControlManager::StandBy()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(STANDBY);

    if (parameters.autorun.as_bool()){
        this->set_parameter(rclcpp::Parameter("autorun",false));
        ToggleState(Run_timer);
    }
    // Check if there is an error
    if (onFault){
        ToggleState(Fault_timer);
    }

}

//================================================
void ControlManager::Run()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(RUN);

    // Publish control module information
    controlControl();

    // Check if joystick to resolve the Warning
    if (navigation_mode == MANUAL_NAVIGATION) {
        if(checkIfJoystick()) {
            updateSolvedWarning(NO_JOYSTICK, NO_JOYSTICK_DESC);
        } else {
            updateErrorWarning(NO_JOYSTICK, NO_JOYSTICK_DESC);
        }
    }

    // Check if there is an error
    if (onFault) {
        ToggleState(Fault_timer);
    }

}

//================================================
void ControlManager::Fault()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(FAULT);
    
    // Introducir las acciones a realizar
    // y la logica de cambio de estado
    // Comprovar error por error  e intentar corregirlos si es posible
    if (critical & BAD_PARAMETERS) {
        if (Initialize()){
            updateSolvedCritical(BAD_PARAMETERS, BAD_PARAMETERS_DESC);
            ToggleState(StandBy_timer);
        }
    }
    // else
    // {
    //     ToggleState(ShutDown_timer);
    // }
}

//================================================
void ControlManager::ShutDown()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(SHUTDOWN);

    // Introducir las acciones a realizar
    // y la logica de cambio de estado
    rclcpp::shutdown();
}

/*****************************************************************************
                FUNCIONES PROPIAS FUNCIONAMIENTO NODO
*****************************************************************************/

//================================================
bool ControlManager::Initialize()
//================================================
{
    // By default start in PAUSE
    navigation_mode = PAUSE_NAVIGATION;

    try {
        this->parameters.autorun = this->get_parameter("autorun");
        this->parameters.manual_pkg = this->get_parameter("manual_pkg");
        this->parameters.autonomous_pkg = this->get_parameter("autonomous_pkg");
        this->parameters.initial_mode = this->get_parameter("initial_mode");
        this->parameters.scan_front = this->get_parameter("scan_front");
        this->parameters.scan_back = this->get_parameter("scan_back");


    } catch (std::runtime_error ex) {
        updateErrorCritical(BAD_PARAMETERS, BAD_PARAMETERS_DESC);
        return false;
    }

    // Create subscription to the two 'cmd_vel' publishers
    manual_sub     = this->create_subscription<geometry_msgs::msg::Twist>(parameters.manual_pkg.as_string()+"/cmd_vel",
                                                                            10, std::bind(&ControlManager::manual_callback, this, _1));
    autonomous_sub = this->create_subscription<geometry_msgs::msg::Twist>(parameters.autonomous_pkg.as_string()+"/cmd_vel",
                                                                            10, std::bind(&ControlManager::autonomous_callback, this, _1));
    
    // Create publisher of 'cmd_vel'
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create publisher of 'control'
    control_publisher = this->create_publisher<ageve_interfaces::msg::Control>("control", 10);

    // Create Laser subscriptions
    std::string robot = this->get_namespace();
    fornt_laser_info = this->create_subscription<ageve_interfaces::msg::LaserDriverHeader>(robot+"/"+parameters.scan_front.as_string(),
                                                                                           10, std::bind(&ControlManager::LaserInfoCallback, this, _1));
    back_laser_info = this->create_subscription<ageve_interfaces::msg::LaserDriverHeader>(robot+"/"+parameters.scan_back.as_string(),
                                                                                           10, std::bind(&ControlManager::LaserInfoCallback, this, _1));
    
    // ADAM Pub and subs
    dig_out_pub = this->create_publisher<ageve_interfaces::msg::DigitalOutputs>("ADAM_Outputs",10);
    dig_in_subs = this->create_subscription<ageve_interfaces::msg::DigitalInputs>(robot+"/ADAM_Driver/ADAM_Inputs", 
                                                                                    10, std::bind(&ControlManager::ADAM_Inputs_Callback, this,_1));
    // Dynamic reconfiguration
    this->set_on_parameters_set_callback(std::bind(&ControlManager::dyn_reconf_callback, this,_1));

    // Temporal:
    if (this->parameters.initial_mode.as_string() == "MANUAL") {
        switchMode(MANUAL_NAVIGATION);
    }
    else if (this->parameters.initial_mode.as_string() == "AUTONOMOUS") {
        switchMode(AUTONOMOUS_NAVIGATION);
    }
    else if (this->parameters.initial_mode.as_string() == "PAUSE") {
        switchMode(PAUSE_NAVIGATION);
    }

    // Okay
    return true;

}

// Callbacks de las dos subscripciones
void ControlManager::manual_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::Twist::SharedPtr new_msg = AdjustVelocity(msg);
    if (navigation_mode == MANUAL_NAVIGATION) {
        cmd_vel_pub->publish(*new_msg);
    }
}

void ControlManager::autonomous_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::Twist::SharedPtr new_msg = AdjustVelocity(msg);
    if (navigation_mode == AUTONOMOUS_NAVIGATION) {
        cmd_vel_pub->publish(*new_msg);
    }
}

// Control the control module
void ControlManager::controlControl() {
    control_msg_.header.stamp = this->now();
    control_msg_.mode = navigation_mode;
    control_publisher->publish(control_msg_);

    //TODO: Añadir logica para ADAM Outputs
}

// Esta función se llamará cuando haya un cambio de valor en el selector de Modo de Navegación
void ControlManager::switchMode(navigation_mode_t mode){
    // Assignar el nuevo valor a la variable privada de modo
    navigation_mode = mode;

    if (navigation_mode == MANUAL_NAVIGATION) {
        RCLCPP_INFO(this->get_logger(), "The navigation mode changed to MANUAL: %s", (parameters.manual_pkg.as_string()+"/cmd_vel").c_str());
        // Comprobar si en Manual hay un joystick conectado
        if (!checkIfJoystick()) {
            updateErrorWarning(NO_JOYSTICK, NO_JOYSTICK_DESC);
        }

    } else if (navigation_mode == AUTONOMOUS_NAVIGATION) {
        RCLCPP_INFO(this->get_logger(), "The navigation mode changed to AUTONOMOUS: %s", (parameters.autonomous_pkg.as_string()+"/cmd_vel").c_str());
    } else if (navigation_mode == PAUSE_NAVIGATION) {
        RCLCPP_INFO(this->get_logger(), "The navigation mode changed to PAUSE");
    }
}

// Esta función sirve para comprobar si hay un joystick conectado.
bool ControlManager::checkIfJoystick() {
    std::string inputs = ageve_utils::utils::executeCommand("ls /dev/input/");
    std::vector<std::string> list_of_inputs;
    boost::split(list_of_inputs, inputs, boost::algorithm::is_any_of("\n"));
    for(auto & inp : list_of_inputs) {
        if (boost::algorithm::contains(inp, "js")) {
            return true;
        }
    }
    return false;
}

//================================================
geometry_msgs::msg::Twist::SharedPtr ControlManager::AdjustVelocity(geometry_msgs::msg::Twist::SharedPtr msg)
//================================================
{
    geometry_msgs::msg::Twist::SharedPtr new_msg = msg;
    if (warn_zone1_front || warn_zone1_back){
        new_msg->linear.x = msg->linear.x/2;
        new_msg->linear.y = msg->linear.y/2;
        new_msg->linear.z = msg->linear.z/2;
        new_msg->angular.x = msg->angular.x/2;
        new_msg->angular.y = msg->angular.y/2;
        new_msg->angular.z = msg->angular.z/2;
    }
    if (warn_zone2_front || warn_zone2_back){
        new_msg->linear.x = msg->linear.x/4;
        new_msg->linear.y = msg->linear.y/4;
        new_msg->linear.z = msg->linear.z/4;
        new_msg->angular.x = msg->angular.x/4;
        new_msg->angular.y = msg->angular.y/4;
        new_msg->angular.z = msg->angular.z/4;
    }
    // If in protected zone adjust cmd_vel
    if (protected_zone_front || protected_zone_back){
        if (this->parameters.initial_mode.as_string() == "AUTONOMOUS"){
            new_msg->linear.x = 0;
            new_msg->linear.y = 0;
            new_msg->linear.z = 0;
            new_msg->angular.x = 0;
            new_msg->angular.y = 0;
            new_msg->angular.z = 0;
        }
        else if (this->parameters.initial_mode.as_string() == "MANUAL"){
            new_msg->linear.x = msg->linear.x/6;
            new_msg->linear.y = msg->linear.y/6;
            new_msg->linear.z = msg->linear.z/6;
            new_msg->angular.x = msg->angular.x/6;
            new_msg->angular.y = msg->angular.y/6;
            new_msg->angular.z = msg->angular.z/6;
        }
    }
    return new_msg;
}

//================================================
void ControlManager::LaserInfoCallback(const ageve_interfaces::msg::LaserDriverHeader::SharedPtr msg)
//================================================
{
    // Check zones
    if (msg->header.frame_id == "laser_front_link"){
            warn_zone1_front = (msg->detection_zone_status & 4 >> 2);
            warn_zone2_front = (msg->detection_zone_status & 2 >> 1);
            protected_zone_front = (msg->detection_zone_status & 1);
            // Check laser zone configuration
            active_zone_set_laser_front = msg->active_protection_zone_set;
    }
    else if (msg->header.frame_id == "laser_back_linki"){
            warn_zone1_back = (msg->detection_zone_status & 4 >> 2);
            warn_zone2_back = (msg->detection_zone_status & 2 >> 1);
            protected_zone_back = (msg->detection_zone_status & 1);
            // Check laser zone configuration
            active_zone_set_laser_back = msg->active_protection_zone_set;
    }
}

//================================================
void ControlManager::ADAM_Inputs_Callback(const ageve_interfaces::msg::DigitalInputs::SharedPtr msg)
//================================================
{
    if (msg->modo_auto) {
        switchMode(MANUAL_NAVIGATION);
    }
    else if (msg->modo_joystick) {
        switchMode(AUTONOMOUS_NAVIGATION);
    }
    else if (msg->modo_standby) {
        switchMode(PAUSE_NAVIGATION);
    }
}

//================================================
rcl_interfaces::msg::SetParametersResult ControlManager::dyn_reconf_callback(const std::vector<rclcpp::Parameter> & parameters_)
//================================================
{
    RCLCPP_INFO(this->get_logger(), "Parameter change request");
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameter change accepted";
    for (const auto & parameter : parameters_){
        try{
            if (parameter.get_name() == "autorun" && 
                parameter.as_bool() != parameters.autorun.as_bool()){
                this->parameters.autorun = parameter;
            }
            else if (parameter.get_name() == "manual_pkg" && 
                parameter.as_string() != parameters.manual_pkg.as_string()){
                this->parameters.manual_pkg = parameter;
            }
            else if (parameter.get_name() == "autonomous_pkg" && 
                parameter.as_string() != parameters.autonomous_pkg.as_string()){
                this->parameters.autonomous_pkg = parameter;
            }
            else if (parameter.get_name() == "initial_mode" && 
                parameter.as_string() != parameters.autonomous_pkg.as_string()){
                this->parameters.initial_mode = parameter;
            }
            else{
                throw "Parameter not found or not available to reconfigure";
            }
            // Node back to StandBy
            ToggleState(Unconfigured_timer);
        }
        catch(std::exception & e){
            RCLCPP_WARN(this->get_logger(),e.what());
            result.successful = false;
            // result.reason = e.what();
            RCLCPP_INFO(this->get_logger(),"Parameter change cancelled", parameter.get_name().c_str());
        }
    }
    return result;
}
