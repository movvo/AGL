/*
 *  Copyright 2023 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

// C++ Standard
#include <iostream>
#include <vector>

//Own hpp
#include "atlas_utils/sm/StateMachineInterface.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "ageve_interfaces/msg/diagnostics.hpp"    
#include "ageve_interfaces/srv/toggle_state.hpp" 

// STATES
#define  UNCONFIGURED   0
#define  STANDBY        1
#define  RUN            2
#define  SHUTDOWN       3
#define  FAULT          4

using namespace std::chrono_literals;
using ToggleStateClientResponse = rclcpp::Client<ageve_interfaces::srv::ToggleState>::SharedFuture;

namespace atlas_utils {
namespace sm {

/*****************************************************************************
                    atlas_utils/sm/StateMachine.hpp
*****************************************************************************/

//======================================
class StateMachine: public StateMachineInterface
//======================================
{
    
    public:
        StateMachine(rclcpp::Node::SharedPtr nh_);
    

        /*---------------------------------
                FUNCIONES PUBLICAS
        ---------------------------------*/


        /*---------------------------------
            FUNCIONES OBTENER STATUS INFO
        ---------------------------------*/   
        void updateFrame_id(std::string value);
        void updateErrorCritical(uint32_t value, std::string description);
        void updateErrorWarning(uint32_t value, std::string description);
        void updateErrorTimeout(uint32_t value, std::string description);
        void updateErrorDescription(std::string value);
        void updateSolvedCritical(uint32_t value, std::string description);
        void updateSolvedWarning(uint32_t value, std::string description);
        void updateSolvedTimeout(uint32_t value, std::string description);
        void updateSolvedDescription(std::string value);
        void updateState(uint8_t value);
    
        /*---------------------------------
            SETTERS / GETTERS
        ---------------------------------*/   
        void SetUnconfiguredTimer(rclcpp::TimerBase::SharedPtr timer);
        void SetStandByTimer(rclcpp::TimerBase::SharedPtr timer);
        void SetFaultTimer(rclcpp::TimerBase::SharedPtr timer);
        void SetRunTimer(rclcpp::TimerBase::SharedPtr timer);
        void SetShutdownTimer(rclcpp::TimerBase::SharedPtr timer);

        rclcpp::TimerBase::SharedPtr GetUnconfiguredTimer() const;
        rclcpp::TimerBase::SharedPtr GetStandByTimer() const;
        rclcpp::TimerBase::SharedPtr GetFaultTimer() const;
        rclcpp::TimerBase::SharedPtr GetRunTimer() const;
        rclcpp::TimerBase::SharedPtr GetShutdownTimer() const;

        /*---------------------------------
            CANCELS / CHECKERS
        ---------------------------------*/   

        void CancelUnconfiguredTimer();
        void CancelStandByTimer();
        void CancelFaultTimer();
        void CancelRunTimer();
        void CancelShutdownTimer();

        bool IsOnFault() const;
        bool IsOnRun() const;
        virtual bool IsCritical(uint32_t errorBit) const;

        uint32_t critical = 0;

    protected:
        /*---------------------------------
             FUNCIONES CAMBIO DE ESTADO
        ---------------------------------*/
        bool ToggleState(rclcpp::TimerBase::SharedPtr timer);

        void ToggleState_srv_callback(const std::shared_ptr<ageve_interfaces::srv::ToggleState::Request> request,
                                            std::shared_ptr<ageve_interfaces::srv::ToggleState::Response> response);

        void Request_Toggle(std::string node_name, uint8_t state2go);


        /*---------------------------------
            VARIABLES PROTEGIDAS
        ---------------------------------*/ 
        rclcpp::CallbackGroup::SharedPtr status_timer_group_;

        // StateMachine Timers
        rclcpp::TimerBase::SharedPtr Unconfigured_timer;
        rclcpp::TimerBase::SharedPtr StandBy_timer;
        rclcpp::TimerBase::SharedPtr Run_timer;
        rclcpp::TimerBase::SharedPtr Fault_timer;
        rclcpp::TimerBase::SharedPtr ShutDown_timer;

        std::string frame_id = "";
        uint32_t warning = 0;
        uint32_t timeout = 0;
        uint8_t state = 0;
        std::vector<uint16_t> nodes_states;
        std::vector<std::string> description;

        // Indicadores para cambio de estado
        bool onFault = false; 
        bool onWarning = false;

        // Request toggle 
        bool request_succes = false;
        void RequestToggle_callback(ToggleStateClientResponse response);
        rclcpp::Client<ageve_interfaces::srv::ToggleState>::SharedPtr toggle_state_public_client;

        //srv
        rclcpp::Service<ageve_interfaces::srv::ToggleState>::SharedPtr toggle_state_srv;
        rclcpp::Client<ageve_interfaces::srv::ToggleState>::SharedPtr toggle_state_client;


    private:
        std::shared_ptr<::rclcpp::Node> nh;
        rclcpp::TimerBase::SharedPtr publisher_timer;
        rclcpp::Publisher<ageve_interfaces::msg::Diagnostics>::SharedPtr diagnostics_pub;
        void DiagnosticsPub_Task_Callback();

        /*---------------------------------
            VARIABLES PRIVADAS
        ---------------------------------*/
        ageve_interfaces::msg::Diagnostics diagnostics_msg;



        /*---------------------------------
            FUNCIONES PRIVADAS
        ---------------------------------*/
        int isPowerOfTwo(uint32_t n);
        int findPosition(uint32_t n);

};
} // namespace sm
} // namespace atlas_utils
#endif // STATE_MACHINE_HPP
