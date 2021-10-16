/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Marti Bolet, Albert Arlà Romero, Iñaki Lorente
   Contact: support.idi@ageve.net
*/


#include "agl_diagnostics/diagnostics_manager.hpp"
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <csignal>
#include <sys/prctl.h> 

// Utils
using namespace ageve_utils::general;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using ServiceResponseFuture =
      rclcpp::Client<ageve_interfaces::srv::Heartbeat>::SharedFuture;
using ToggleStateClientResponse = rclcpp::Client<ageve_interfaces::srv::ToggleState>::SharedFuture;

/*****************************************************************************
							DIAGNOSTICS CONSTRUCTOR
*****************************************************************************/

DiagnosticManager::DiagnosticManager(std::string filename) : Node("diagnostic_manager"),
															 StateMachine(std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {}))
{
	namespace_ = this->get_effective_namespace();
	filename_ = filename;
	boost::replace_all(namespace_, "/", "");

    // Definir los State Machine timers a la frequencia deseada
    // y vincularlos a sus callback
    Unconfigured_timer = this->create_wall_timer(500ms, std::bind(&DiagnosticManager::Unconfigured,this));
    StandBy_timer = this->create_wall_timer(500ms, std::bind(&DiagnosticManager::StandBy,this));
    Run_timer = this->create_wall_timer(500ms, std::bind(&DiagnosticManager::Run,this));
    Fault_timer = this->create_wall_timer(500ms, std::bind(&DiagnosticManager::Fault,this));
    ShutDown_timer = this->create_wall_timer(500ms, std::bind(&DiagnosticManager::ShutDown,this));
    
    // Cancel all timers except the first one
    StandBy_timer->cancel();
    Run_timer->cancel();
    Fault_timer->cancel();
    ShutDown_timer->cancel();
}

// Destructor
DiagnosticManager::~DiagnosticManager(){
	prctl(PR_SET_PDEATHSIG, SIGHUP);
}

/*****************************************************************************
                    State Machine: FUNCIONES ESTADO NODO/ROBOT
*****************************************************************************/

//================================================
void DiagnosticManager::Unconfigured()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(UNCONFIGURED);
	RCLCPP_DEBUG(this->get_logger(),"Robot Unconfigured");
    // Introducir las acciones a realizar
    // y la logica de cambio de estado

	// Initialize the parameters and subscribe to all status topics of the robot
	if (Initialize() && Subscribe()){
		ToggleState(StandBy_timer);
	}
}

//================================================
void DiagnosticManager::StandBy()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(STANDBY);

    RCLCPP_DEBUG(this->get_logger(),"On Stand By");
    if (params.autorun.as_bool()){
        // Reset autorun to false
		// TODO: Implementar logica de si todos los nodos configurados
		this->set_parameter(rclcpp::Parameter("autorun",false));
		ToggleState(Run_timer);
		

    }
    else{
		// Ir actualizando los errores para enviarlos por el mensaje
		// Status, info para el fleetmanager.
		updateRobotStatus();
    }
    if (onFault){
        ToggleState(Fault_timer);
    }
}

//================================================
void DiagnosticManager::Run()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(RUN);
	RCLCPP_DEBUG(this->get_logger(),"ROBOT ON RUN");
	RobotRun();
	merge_publish_status();
	// Ir actualizando los errores para enviarlos por el mensaje
	// Status, info para el fleetmanager.
	updateRobotStatus();
	if (onFault){
        ToggleState(Fault_timer);
    }

}

//================================================
void DiagnosticManager::Fault()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(FAULT);

    // Introducir las acciones a realizar
    // y la logica de cambio de estado
    RCLCPP_DEBUG(this->get_logger(),"ROBOT ON FAULT");

	updateRobotStatus();
	if (!onFault){
		ToggleState(StandBy_timer);
	}
}

//================================================
void DiagnosticManager::ShutDown()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(SHUTDOWN);
	RCLCPP_DEBUG(this->get_logger(),"ROBOT ON SHUTDOWN");
	Release();
    // Introducir las acciones a realizar
    // y la logica de cambio de estado

}

/*****************************************************************************
                FUNCIONES PROPIAS FUNCIONAMIENTO DEL DIAGNOSTICS
*****************************************************************************/

// Initialize parameters
//Return 1 if everything OK
//================================================
bool DiagnosticManager::Initialize()
//================================================
{
	try{
    // Declare parameters
    this->declare_parameter("timeout");
	this->declare_parameter("nodes_list");
	this->declare_parameter("autorun");

	// Get parameters
	this->get_parameter("timeout", params.timeout);
	this->get_parameter("nodes_list", params.nodes_list);
	this->get_parameter("autorun",params.autorun);
	}
	catch (std::runtime_error ex){
        RCLCPP_ERROR(this->get_logger(),"Error init parameters");
        return false;
	}
	params.ip = getIp(); //ageve_utils::general::getIp()
	if (params.ip.empty()) {
		RCLCPP_WARN(this->get_logger(), "Error getting the IP of the local interface");
	}

	// Initialize the size of nodes_states
	diagnostics_msg_.nodes_states = std::vector<uint8_t>(params.nodes_list.size(), 0);

	// Subscribe to Fleet Manager updater
	fleet_manager_updated = this->create_subscription<std_msgs::msg::Empty>("/fleet_manager/update_fleet", 10, 
									std::bind(&DiagnosticManager::InformFM, this, _1));
	fleet_manager_client  = this->create_client<ageve_interfaces::srv::Heartbeat>("/fleet_manager/update_fleet_srv");

	// Merged Status publisher init
	StatusMerged_pub = this->create_publisher<ageve_interfaces::msg::ErrorsLog>("MergedStatus", 10);
	//Srv to send yaml
	yaml_transfer_srv = this->create_service<ageve_interfaces::srv::YamlTransfer>("fleet_manager/yaml_file_request",std::bind(&DiagnosticManager::send_yaml_callback, this, _1,_2));
	yaml_recieve_srv = this->create_service<ageve_interfaces::srv::YamlTransfer>("fleet_manager/yaml_file_update",std::bind(&DiagnosticManager::recieve_yaml_callback, this, _1,_2));
	return true;
}

// Manage subscriptions to all the list of "params.yaml"
//================================================
bool DiagnosticManager::Subscribe()
//================================================
{
	// In the following line is defined a generic function to use more than the placeholders params
	// that can accept std::bind.
	std::function<void(std::shared_ptr<ageve_interfaces::msg::Diagnostics>)> fnc;
	// For each node_name in the list
	for (int i=0; i<params.nodes_list.size(); i++) {
		// Register the callback with the extra parameter of 'node_name'
		fnc = std::bind(&DiagnosticManager::status_callback, this, _1, params.nodes_list[i]);
		
		// Create subscription
		subscriptions_.emplace_back(this->create_subscription<ageve_interfaces::msg::Diagnostics>(
			params.nodes_list[i] + "/status", 10, fnc)
		);
		RCLCPP_INFO(this->get_logger(),"Subscribed to '%s'",params.nodes_list[i].c_str());
	}
	return true;
}

//================================================
void DiagnosticManager::Release()
//================================================
{
	for (int i=0; i<params.nodes_list.size(); i++) {
		try{
			Request_Toggle(params.nodes_list[i],SHUTDOWN);
		}
		catch (rclcpp::exceptions::RCLError err){
			RCLCPP_ERROR(this->get_logger(), "Toggle state to node '%s' failed, service not available", params.nodes_list[i].c_str());
		}
	}
	rclcpp::shutdown();
}

// Callback of publisher
//================================================
void DiagnosticManager::updateRobotStatus()
//================================================
{
	// Check all diagnostics in the memory [nodes_diagnostics]
	// Reset states
	diagnostics_msg_.error.warning  = 0;
	diagnostics_msg_.error.critical = 0;
	diagnostics_msg_.error.timeout = 0;

	std::string node_name; // Temporal var
	for (int i=0; i<params.nodes_list.size(); i++) {
		node_name = params.nodes_list[i];
		// CHECK for Errors or Warnings
		if (nodes_diagnostics[node_name].warnings > 0) {
			diagnostics_msg_.error.warning |= (1<<i);
		}
		if (nodes_diagnostics[node_name].errors > 0) {
			diagnostics_msg_.error.critical |= (1<<i);
		}
		// If timeout identity with the corresponding bit
		if (!nodes_diagnostics[node_name].connected) {
			diagnostics_msg_.error.timeout |= (1<<i);
		}

		// TIMEOUT of Nodes Status
		// To check that are arriving new messages:
		if(nodes_diagnostics[node_name].new_msg){
			// If there is a new message, reset the variables
			nodes_diagnostics[node_name].new_msg   = false;
			nodes_diagnostics[node_name].count     = 0;
			nodes_diagnostics[node_name].connected = true;
			diagnostics_msg_.error.timeout &= (0<<i);
		} else {
			// If the message isnt new, count +1
			nodes_diagnostics[node_name].count += 1;
		}
		if (nodes_diagnostics[node_name].count > params.timeout) {
			nodes_diagnostics[node_name].connected = false;
			RCLCPP_ERROR(this->get_logger(), "Stopped reciving node status: %s", node_name.c_str());
		}

		// Send state information about each node
		diagnostics_msg_.nodes_states[i] = nodes_diagnostics[node_name].state;
	}

	// Actualizamos las variables de la classe StateMachine para enviar mensaje Status
	critical = diagnostics_msg_.error.critical;
	warning = diagnostics_msg_.error.warning;
	timeout = diagnostics_msg_.error.timeout;
	nodes_states = diagnostics_msg_.nodes_states;
	if (critical > 0 || timeout > 0){
		onFault = true;
	}
	else {
		onFault = false;
	}
}

//================================================
void DiagnosticManager::merge_publish_status()
//================================================
{
	ageve_interfaces::msg::LogMsg log_msg;
	for (int i=0; i<params.nodes_list.size(); i++) {
		log_msg.name = params.nodes_list[i];
		log_msg.state = nodes_diagnostics[log_msg.name].state;
		log_msg.error.warning = nodes_diagnostics[log_msg.name].warnings;
		log_msg.error.critical = nodes_diagnostics[log_msg.name].errors;
		log_msg.error.description = nodes_diagnostics[log_msg.name].err_description;
		log_msg.timestamp = nodes_diagnostics[log_msg.name].stamp;
		StatusMerged_msg.package.push_back(log_msg);
	}
	StatusMerged_msg.header.stamp = this->now();
	StatusMerged_pub->publish(StatusMerged_msg);
}

// Callback for manage subscriptions
//================================================
void DiagnosticManager::status_callback(const ageve_interfaces::msg::Diagnostics::SharedPtr msg, std::string node_name) 
//================================================
{
	// std::cout << "status_callback called from node: " << namespace_ << "/" << node_name << " Warn: " << msg->error.warning << std::endl;
	nodes_diagnostics[node_name].state     = msg->state;
	nodes_diagnostics[node_name].warnings  = msg->error.warning;
	nodes_diagnostics[node_name].errors    = msg->error.critical;
	nodes_diagnostics[node_name].err_description = msg->error.description;
	nodes_diagnostics[node_name].stamp = msg->header.stamp;

	// To check that are arriving new messages
	nodes_diagnostics[node_name].new_msg = true;
}

//================================================
void DiagnosticManager::InformFM(const std_msgs::msg::Empty::SharedPtr msg) 
//================================================
{
	// TODO: Call the service
	auto request = std::make_shared<ageve_interfaces::srv::Heartbeat::Request>();
	request->ip = params.ip;
	request->vehicle_namespace = namespace_;
	request->state = diagnostics_msg_.state;
	request->node_list = params.nodes_list;

	while (!fleet_manager_client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
	}

	auto result = fleet_manager_client->async_send_request(request,std::bind(&DiagnosticManager::ServiceCallback,this,_1));
}

//================================================
void DiagnosticManager::ServiceCallback(ServiceResponseFuture response)
//================================================
{
	RCLCPP_INFO(this->get_logger(), "Sum: %ld", response.get()->ok);
}

//================================================
void DiagnosticManager::send_yaml_callback(const std::shared_ptr<ageve_interfaces::srv::YamlTransfer::Request> request,
							std::shared_ptr<ageve_interfaces::srv::YamlTransfer::Response> response)
//================================================
{
	std::string yaml2send = get_params_yaml();
	response->yaml_file_resp = yaml2send;
}

//================================================
void DiagnosticManager::recieve_yaml_callback(const std::shared_ptr<ageve_interfaces::srv::YamlTransfer::Request> request,
							std::shared_ptr<ageve_interfaces::srv::YamlTransfer::Response> response)
//================================================
{
	std::string resp = "ok";
	response->yaml_file_resp = resp;
	std::string yaml_updated = request->yaml_file_req;
	YAML::Node yaml2update = YAML::Load(yaml_updated);
	std::string directory = executeCommand(std::string("echo $(dirname ")+filename_+std::string(")"));
	boost::replace_all(directory,"\n","");
	std::string exec_directory = executeCommand(std::string("cd ")+directory+std::string(" && echo $(pwd)"));
	boost::replace_all(exec_directory,"\n","");
	std::ofstream fout(exec_directory+"/../../../../params_temp.yaml"); 
	fout << yaml2update; // dump it back into the file
}

//Parsing Params.yaml file
//================================================
std::string DiagnosticManager::get_params_yaml()
//================================================
{
	std::string directory = executeCommand(std::string("echo $(dirname ")+filename_+std::string(")"));
	boost::replace_all(directory,"\n","");
	std::string exec_directory = executeCommand(std::string("cd ")+directory+std::string(" && echo $(pwd)"));
	boost::replace_all(exec_directory,"\n","");
	YAML::Node config = YAML::LoadFile(exec_directory+"/../../../agl_bringup/share/agl_bringup/config/params.yaml");
	std::string yaml2str = YAML::Dump(config);
	return yaml2str;
}

//================================================
std::string DiagnosticManager::executeCommand(std::string command) 
//================================================
{
    char buffer[128];
    std::string result = "";
    // Open pipe to file
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        return "popen failed!";
    }
    // read till end of process:
    while (!feof(pipe)) {
        // use buffer to read and add to result
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}

//================================================
void DiagnosticManager::RobotRun()
//================================================
{
	switch (RunStateControl.run_state){
		case WAITING_ORDERS:{
			break;
		}
		case CHARGING:{
			break;
		}
		case ON_MISSION:{
			break;
		}
		default:{

		}
	}
}


/*****************************
	RECONFIGURACIÓN DINÁMICA
******************************/

//================================================
rcl_interfaces::msg::SetParametersResult DiagnosticManager::dyn_reconf_callback(const std::vector<rclcpp::Parameter> & parameters)
//================================================
{
    RCLCPP_INFO(this->get_logger(), "Parameter change request");
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "The reason could not be allowed";
    for (const auto & parameter : parameters){
        try{
			if (parameter.get_name() == "autorun" && parameter.as_bool() != this->params.autorun.get_value<bool>()){
                result.successful = true;
                result.reason = "Parameter change accepted";
                RCLCPP_INFO(this->get_logger(), "Parameter change accepted",parameter.get_name().c_str());
                this->params.autorun = parameter;
                RCLCPP_INFO(this->get_logger(),"Parameter '%s' changed",parameter.get_name().c_str());

			}
            else{
                throw std::invalid_argument("Parameter not found or not available to reconfigure");
            }
            // Volvemos a etapa Unconfigured
            ToggleState(StandBy_timer);
        }
        catch(std::exception & e){
            RCLCPP_WARN(this->get_logger(),e.what());
            result.reason = e.what();
            RCLCPP_INFO(this->get_logger(),"Parameter change cancelled",parameter.get_name().c_str());
        }
		catch (const std::invalid_argument& e){
            RCLCPP_WARN(this->get_logger(),"Error changing parameter: '%s'",e.what());
        }
    }
    return result;
}

/*****************************
	MAIN
******************************/

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DiagnosticManager>(std::string(argv[0])));
	rclcpp::shutdown();
	return 0;
}
