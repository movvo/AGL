/*
 *  Copyright 2023 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#include "atlas_utils/sm/StateMachine.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using namespace atlas_utils::sm;

//================================================
StateMachine::StateMachine (rclcpp::Node::SharedPtr nh_)
//================================================
{
    // Save node handle
    nh = nh_;
    std::string child_node = nh->get_name();
    // status_timer_group_ = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    diagnostics_pub = nh->rclcpp::Node::create_publisher<atlas_interfaces::msg::Diagnostics>("~/status",10);
    // publisher_timer = nh->rclcpp::Node::create_wall_timer(100ms,std::bind(&StateMachine::DiagnosticsPub_Task_Callback, this),status_timer_group_);
    publisher_timer = nh->rclcpp::Node::create_wall_timer(100ms,std::bind(&StateMachine::DiagnosticsPub_Task_Callback, this));
    toggle_state_srv = nh->rclcpp::Node::create_service<atlas_interfaces::srv::ToggleState>(child_node+"/ToggleState",
                                                                                            std::bind(&StateMachine::ToggleState_srv_callback,this,_1,_2));

}

//================================================
void StateMachine::DiagnosticsPub_Task_Callback()
//================================================
{
    diagnostics_msg.header.stamp = nh->now();
    diagnostics_msg.header.frame_id = frame_id;
    diagnostics_msg.error.warning = warning;
    diagnostics_msg.error.critical = critical;
    diagnostics_msg.error.timeout = timeout;
    diagnostics_msg.error.description = description;
    diagnostics_msg.state = state;
    diagnostics_msg.nodes_states = nodes_states;
    diagnostics_pub->publish(diagnostics_msg);
}

/*****************************
 *  GETTERS / SETTERS 
******************************/

void StateMachine::SetUnconfiguredTimer(rclcpp::TimerBase::SharedPtr timer)
{
    Unconfigured_timer = timer;
}

void StateMachine::SetStandByTimer(rclcpp::TimerBase::SharedPtr timer)
{
    StandBy_timer = timer;
}

void StateMachine::SetFaultTimer(rclcpp::TimerBase::SharedPtr timer)
{
    Fault_timer = timer;

}

void StateMachine::SetRunTimer(rclcpp::TimerBase::SharedPtr timer)
{
    Run_timer = timer;

}

void StateMachine::SetShutdownTimer(rclcpp::TimerBase::SharedPtr timer)
{
    ShutDown_timer = timer;
}

rclcpp::TimerBase::SharedPtr StateMachine::GetUnconfiguredTimer() const
{
    return Unconfigured_timer;
}

rclcpp::TimerBase::SharedPtr StateMachine::GetStandByTimer() const
{
    return StandBy_timer;
}

rclcpp::TimerBase::SharedPtr StateMachine::GetFaultTimer() const
{
    return Fault_timer;
}

rclcpp::TimerBase::SharedPtr StateMachine::GetRunTimer() const
{
    return Run_timer;
}

rclcpp::TimerBase::SharedPtr StateMachine::GetShutdownTimer() const
{
    return ShutDown_timer;
}


/*****************************
 *  CANCELS / CHECKERS
******************************/

void StateMachine::CancelUnconfiguredTimer()
{
    Unconfigured_timer->cancel();
}

void StateMachine::CancelStandByTimer()
{
    StandBy_timer->cancel();
}

void StateMachine::CancelFaultTimer()
{
    Fault_timer->cancel();
}

void StateMachine::CancelRunTimer()
{
    Run_timer->cancel();
}

void StateMachine::CancelShutdownTimer()
{
    ShutDown_timer->cancel();
}

bool StateMachine::IsOnFault() const
{
    return onFault;
}

bool StateMachine::IsOnRun() const
{
    if (state == RUN){
        return true;
    }
    return false;
}

bool StateMachine::IsCritical(uint32_t errorBit) const
{
    return critical & errorBit;
}

/*****************************
 *  FUNCIONES CAMBIO DE ESTADO
******************************/

//================================================
bool StateMachine::ToggleState(rclcpp::TimerBase::SharedPtr timer)
//================================================
{
    if (!Run_timer->is_canceled()){
        Run_timer->cancel();
        timer->reset();
        return true;
        // timer = nh->create_wall_timer(frequency, std::bind(this::))
    }
    else if (!Fault_timer->is_canceled()){
        Fault_timer->cancel();
        timer->reset();
        return true;
    }
    else if (!StandBy_timer->is_canceled()){
        StandBy_timer->cancel();
        timer->reset();
        return true;
    }
    else if (!ShutDown_timer->is_canceled()){
        ShutDown_timer->cancel();
        timer->reset();
        return true;
    }
    else if (!Unconfigured_timer->is_canceled()){
        Unconfigured_timer->cancel();
        timer->reset();
        return true;
    }
    return false;
}

//================================================
void StateMachine::ToggleState_srv_callback(const std::shared_ptr<atlas_interfaces::srv::ToggleState::Request> request,
                                            std::shared_ptr<atlas_interfaces::srv::ToggleState::Response> response)
//================================================
{
    if (request->state == UNCONFIGURED){
        if (!Run_timer->is_canceled()){
            Run_timer->cancel();
            Unconfigured_timer->reset();
            response->ok = 1;
        }
        else if (!Fault_timer->is_canceled()){
            Fault_timer->cancel();
            Unconfigured_timer->reset();
            response->ok = 1;
        }
        else if (!StandBy_timer->is_canceled()){
            StandBy_timer->cancel();
            Unconfigured_timer->reset();
            response->ok = 1;
        }
        else{
            response->ok = 0;
        }
    }
    else if (request->state == STANDBY){
        if (!Run_timer->is_canceled()){
            Run_timer->cancel();
            StandBy_timer->reset();
            response->ok = 1;
        }
        else if (!Unconfigured_timer->is_canceled()){
            Unconfigured_timer->cancel();
            StandBy_timer->reset();
            response->ok = 1;
        }
        else{
            response->ok = 0;
        }
    }
    else if (request->state == RUN){
        if (!StandBy_timer->is_canceled()){
            StandBy_timer->cancel();
            Run_timer->reset();
            response->ok = 1;
        }
        else{
            response->ok = 0;
        }
    }
    else if (request->state == SHUTDOWN){
        if (!StandBy_timer->is_canceled()){
            StandBy_timer->cancel();
            ShutDown_timer->reset();
            response->ok = 1;
        }
        else if (!Unconfigured_timer->is_canceled()){
            Unconfigured_timer->cancel();
            ShutDown_timer->reset();
            response->ok = 1;
        }
        else if (!Run_timer->is_canceled()){
            Run_timer->cancel();
            ShutDown_timer->reset();
            response->ok = 1;
        }
        else if (!Fault_timer->is_canceled()){
            Fault_timer->cancel();
            ShutDown_timer->reset();
            response->ok = 1;
        }
        else{
            response->ok = 0;
        }
    }
    else if (request->state == FAULT){
        if (!StandBy_timer->is_canceled()){
            StandBy_timer->cancel();
            Fault_timer->reset();
            response->ok = 1;
        }
        else if (!Unconfigured_timer->is_canceled()){
            Unconfigured_timer->cancel();
            Fault_timer->reset();
            response->ok = 1;
        }
        else if (!Run_timer->is_canceled()){
            Run_timer->cancel();
            Fault_timer->reset();
            response->ok = 1;
        }
        else{
            response->ok = 0;
        }
    }
    else{
        response->ok = 0;
    }
}

//================================================
void StateMachine::Request_Toggle(std::string node_name,uint8_t state2go)
//================================================
{
	// Creamos un ToggleState client para cada nodo en la lista
    request_succes = false;
	toggle_state_client = nh->create_client<atlas_interfaces::srv::ToggleState>(node_name+"/ToggleState");
	auto request_toggle = std::make_shared<atlas_interfaces::srv::ToggleState::Request>();
	request_toggle->state = state2go;
	RCLCPP_INFO(nh->get_logger(),"Request Toggle state of '%s' to state '%i'",node_name.c_str(),state2go);
	auto toggle_result = toggle_state_client->async_send_request(request_toggle,std::bind(&StateMachine::RequestToggle_callback,this,_1));
}
// //================================================
// void Request_Toggle(rclcpp::Node::SharedPtr node_handle,std::string node_name, uint8_t state2go)
// //================================================
// {
//     // Creamos un ToggleState client para cada nodo en la lista
//     toggle_succes = false;
// 	toggle_state_public_client = node_handle->create_client<atlas_interfaces::srv::ToggleState>(node_name+"/ToggleState");
// 	auto request_toggle = std::make_shared<atlas_interfaces::srv::ToggleState::Request>();
// 	request_toggle->state = state2go;
// 	RCLCPP_INFO(node_handle->get_logger(),"Request Toggle state of '%s' to state '%i'",node_name.c_str(),state2go);
// 	auto toggle_result = toggle_state_public_client->async_send_request(request_toggle,std::bind(&StateMachine::RequestToggle_callback,node_handle,_1));
// }

//================================================
void StateMachine::RequestToggle_callback(ToggleStateClientResponse response)
//================================================
{
	if (response.get()->ok == 1){
		RCLCPP_INFO(nh->get_logger(),"Toggle State Request Succesful");
        request_succes = true;
	}
	else{
		RCLCPP_INFO(nh->get_logger(),"Toggle State Request Failed");
	}
}




/*****************************
 *  FUNCIONES OBTENER STATUS INFO
******************************/   

//================================================
void StateMachine::updateFrame_id(std::string value)
//================================================
{
    frame_id = value;
}

//================================================
void StateMachine::updateErrorCritical(uint32_t value, std::string description)
//================================================
{
    onFault = true;
    critical = critical | value;
    updateErrorDescription(description);
}

//================================================
void StateMachine::updateErrorWarning(uint32_t value, std::string description)
//================================================
{
    warning = warning | value;
    onWarning = true;
    updateErrorDescription(description);
}

//================================================
void StateMachine::updateErrorTimeout(uint32_t value, std::string description)
//================================================
{
    timeout = timeout | value;
    updateErrorDescription(description);
}

//================================================
void StateMachine::updateErrorDescription(std::string value)
//================================================
{
    // If description is already in array, dont add it again
    if (std::find(description.begin(), description.end(), value) != description.end())
        return;    
    description.push_back(value);
}

//================================================
void StateMachine::updateSolvedCritical(uint32_t value, std::string description)
//================================================
{
    int bit_pos = findPosition(value);
    critical &= ~(1UL << (bit_pos-1));
    if (critical == 0){
        onFault = false;
    }
    updateSolvedDescription(description);
}

//================================================
void StateMachine::updateSolvedWarning(uint32_t value, std::string description)
//================================================
{
    int bit_pos = findPosition(value);
    warning &= ~(1UL << (bit_pos-1));
    if (warning == 0){
        onWarning = false;
    }
    updateSolvedDescription(description);
}

//================================================
void StateMachine::updateSolvedTimeout(uint32_t value, std::string description)
//================================================
{
    int bit_pos = findPosition(value);
    timeout &= ~(1UL << (bit_pos-1));
    updateSolvedDescription(description);
}

//================================================
void StateMachine::updateSolvedDescription(std::string value)
//================================================
{
    if (std::find(description.begin(), description.end(), value) != description.end()){
        description.erase(std::remove(description.begin(),description.end(),value));
    }
}

//================================================
void StateMachine::updateState(uint8_t value)
//================================================
{
    state = value;
}


// Funciones utiles
//================================================
int StateMachine::isPowerOfTwo(uint32_t n)
//================================================
{
    return n && (!(n & (n - 1)));
}
 
// Returns position of the only set bit in 'n'
//================================================
int StateMachine::findPosition(uint32_t n)
//================================================
{
    if (!isPowerOfTwo(n))
        return -1;
 
    uint32_t i = 1, pos = 1;
 
    // Iterate through bits of n till we find a set bit
    // i&n will be non-zero only when 'i' and 'n' have a set bit
    // at same position
    while (!(i & n)) {
        // Unset current bit and set the next bit in 'i'
        i = i << 1;
 
        // increment position
        ++pos;
    }
 
    return pos;
}
