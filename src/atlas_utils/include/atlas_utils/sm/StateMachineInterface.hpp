/*
 *  Copyright 2023 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#ifndef STATE_MACHINE_INTERFACE_HPP
#define STATE_MACHINE_INTERFACE_HPP

// C++ Standard
#include <string>
#include <memory>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "atlas_interfaces/srv/toggle_state.hpp" 


namespace atlas_utils {
namespace sm {

/*****************************************************************************
                    atlas_utils/sm/StateMachine.hpp
*****************************************************************************/

//======================================
class StateMachineInterface
//======================================
{
    public:
        virtual ~StateMachineInterface() {};
        /*---------------------------------
            FUNCIONES OBTENER STATUS INFO
        ---------------------------------*/   
        virtual void updateFrame_id(std::string value) = 0;
        virtual void updateErrorCritical(uint32_t value, std::string description) = 0;
        virtual void updateErrorWarning(uint32_t value, std::string description) = 0;
        virtual void updateErrorTimeout(uint32_t value, std::string description) = 0;
        virtual void updateErrorDescription(std::string value) = 0;
        virtual void updateSolvedCritical(uint32_t value, std::string description) = 0;
        virtual void updateSolvedWarning(uint32_t value, std::string description) = 0;
        virtual void updateSolvedTimeout(uint32_t value, std::string description) = 0;
        virtual void updateSolvedDescription(std::string value) = 0;
        virtual void updateState(uint8_t value) = 0;

        /*---------------------------------
            SETTERS / GETTERS
        ---------------------------------*/   
        virtual void SetUnconfiguredTimer(rclcpp::TimerBase::SharedPtr timer) = 0;
        virtual void SetStandByTimer(rclcpp::TimerBase::SharedPtr timer) = 0;
        virtual void SetFaultTimer(rclcpp::TimerBase::SharedPtr timer) = 0;
        virtual void SetRunTimer(rclcpp::TimerBase::SharedPtr timer) = 0;
        virtual void SetShutdownTimer(rclcpp::TimerBase::SharedPtr timer) = 0;

        virtual rclcpp::TimerBase::SharedPtr GetUnconfiguredTimer() const = 0;
        virtual rclcpp::TimerBase::SharedPtr GetStandByTimer() const = 0;
        virtual rclcpp::TimerBase::SharedPtr GetFaultTimer() const = 0;
        virtual rclcpp::TimerBase::SharedPtr GetRunTimer() const = 0;
        virtual rclcpp::TimerBase::SharedPtr GetShutdownTimer() const = 0;

        /*---------------------------------
            CANCELS / CHECKERS
        ---------------------------------*/
        virtual void CancelUnconfiguredTimer() = 0;
        virtual void CancelStandByTimer() = 0;
        virtual void CancelFaultTimer() = 0;
        virtual void CancelRunTimer() = 0;
        virtual void CancelShutdownTimer() = 0;

        virtual bool IsOnFault() const = 0;
        virtual bool IsOnRun() const = 0;
        virtual bool IsCritical(uint32_t errorBit) const = 0;
        
        /*---------------------------------
             FUNCIONES CAMBIO DE ESTADO
        ---------------------------------*/
        virtual bool ToggleState(rclcpp::TimerBase::SharedPtr timer) = 0;
        virtual void ToggleState_srv_callback(
            const std::shared_ptr<atlas_interfaces::srv::ToggleState::Request> request,
            std::shared_ptr<atlas_interfaces::srv::ToggleState::Response> response) = 0;
        virtual void Request_Toggle(std::string node_name, uint8_t state2go) = 0;

};
} // namespace sm
} // namespace atlas_utils
#endif // STATE_MACHINE_INTERFACE_HPP
