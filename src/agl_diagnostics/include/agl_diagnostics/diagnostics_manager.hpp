/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Marti Bolet, Albert Arlà Romero, Iñaki Lorente
   Contact: support.idi@ageve.net
*/


#ifndef DIAGNOSTICS_MAIN_H
#define DIAGNOSTICS_MAIN_H

// General
#include <chrono>
#include <string>
#include <boost/algorithm/string/replace.hpp>
#include "yaml-cpp/yaml.h"
#include <boost/algorithm/string/replace.hpp>
#include <cstdlib>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"


// Diagnostics message
#include "ageve_interfaces/msg/diagnostics.hpp"
#include "ageve_interfaces/srv/heartbeat.hpp"
#include "ageve_interfaces/srv/yaml_transfer.hpp"
#include "ageve_interfaces/srv/toggle_state.hpp" 
#include "ageve_interfaces/msg/errors_log.hpp"
#include "ageve_interfaces/msg/log_msg.hpp"
#include "ageve_interfaces/msg/error.hpp"

// Utils
#include "ageve_utils/general/general_funcs.hpp"
#include "ageve_utils/general/system_funcs.hpp"
#include "ageve_utils/sm/StateMachine.hpp"


class DiagnosticManager : public rclcpp::Node, public ageve_utils::sm::StateMachine{
    public:
        // General Methods
        DiagnosticManager(std::string filename);
        virtual ~DiagnosticManager();

        /*****************************
         *  FUNCIONES ESTADO DEL ROBOT
        ******************************/
        void Unconfigured();
        void StandBy();
        void Run();
        void Fault();
        void ShutDown();

        /*****************************
         *  FUNCIONES PROPIAS DEL NODO
        ******************************/
        // Inicialización
        bool Initialize();
        // Shutdown
        void Release();
        // Subscribers
        void status_callback(const ageve_interfaces::msg::Diagnostics::SharedPtr msg, std::string node_name);
        bool Subscribe();

        // onRun ir actualizando el mensaje del status
        void updateRobotStatus();

        // Callbacks de servicios
        void InformFM(const std_msgs::msg::Empty::SharedPtr msg);
        using ServiceResponseFuture =
            rclcpp::Client<ageve_interfaces::srv::Heartbeat>::SharedFuture;
	    void ServiceCallback(ServiceResponseFuture response);

        void send_yaml_callback(const std::shared_ptr<ageve_interfaces::srv::YamlTransfer::Request> request,
                                std::shared_ptr<ageve_interfaces::srv::YamlTransfer::Response> response);
        void recieve_yaml_callback(const std::shared_ptr<ageve_interfaces::srv::YamlTransfer::Request> request,
                                std::shared_ptr<ageve_interfaces::srv::YamlTransfer::Response> response);
        //User functions
        std::string get_params_yaml();
        std::string executeCommand(std::string command);

        /*****************************
         *  VARIABLES PÚBLICAS
        ******************************/
        std::string filename_;

        // Subscribe to FleetManager update event
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr fleet_manager_updated;
        rclcpp::Client<ageve_interfaces::srv::Heartbeat>::SharedPtr fleet_manager_client;

        std::vector<rclcpp::Subscription<ageve_interfaces::msg::Diagnostics>::SharedPtr> subscriptions_;

    private:

        /*****************************
         *  RECONFIGURACIÓN DINÁMICA
        ******************************/
        rcl_interfaces::msg::SetParametersResult dyn_reconf_callback(const std::vector<rclcpp::Parameter> & parameters);

        /*****************************
         *  FUNCIONES PROPIAS DEL NODO
        ******************************/
        void RobotRun();
        void merge_publish_status();

        /*****************************
         *  VARIABLES PRIVADAS
        ******************************/
        std::shared_ptr<rclcpp::Node> nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
        std::string namespace_;  // Namespace of the vehicle
        typedef struct {         // Parameters of the node
            bool publish;
            int timeout;
            std::string ip;
            std::vector<std::string> nodes_list;
            rclcpp::Parameter autorun;
        } Parameters;
        Parameters params;

        // Client para el servicio ToggleState
        // rclcpp::Client<ageve_interfaces::srv::ToggleState>::SharedPtr ToggleState_client;

        // Parameters of each node
        typedef struct {
            bool new_msg;   // To know if are arriving new messages
            uint8_t count;  // Number of times without new message
            bool connected;
            uint8_t state;
            uint32_t warnings;
            uint32_t errors;
            rclcpp::Time stamp;
            std::vector<std::string> err_description;
        } DiagnosticMsg;
        std::map<std::string, DiagnosticMsg> nodes_diagnostics {};
        ageve_interfaces::msg::Diagnostics diagnostics_msg_;
        rclcpp::Service<ageve_interfaces::srv::YamlTransfer>::SharedPtr yaml_transfer_srv;
        rclcpp::Service<ageve_interfaces::srv::YamlTransfer>::SharedPtr yaml_recieve_srv;
        rclcpp::Publisher<ageve_interfaces::msg::ErrorsLog>::SharedPtr StatusMerged_pub;
        ageve_interfaces::msg::ErrorsLog StatusMerged_msg;

        // Estados del robot en RUN
        typedef enum
        {       
            WAITING_ORDERS=0,
            CHARGING,
            ON_MISSION
        } RUN_STATES;
        struct RunStateControl_struct {
            RUN_STATES run_state;  
        };
        struct RunStateControl_struct RunStateControl {
            .run_state = WAITING_ORDERS
        };
};


#endif // DIAGNOSTICS_MAIN_H