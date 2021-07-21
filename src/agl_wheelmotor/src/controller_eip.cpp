
#include "me00_wheelmotor/controller_eip.h"
#include "me00_wheelmotor/anybus_constants.h"

namespace roboteq {

// Constructor
Controller_EIP::Controller_EIP(std::shared_ptr<rclcpp::Node> &nh)
    : nh_(nh)
    , connection_num_(-1)
    , sequence_num_(1) {
    Initialize();
}
Controller_EIP::~Controller_EIP() {
    Release();
}

// Initialize
bool Controller_EIP::Initialize() {
    // Declare parameters
    nh_->declare_parameter<int>("controller_eip.udp_port", 2222);
    nh_->declare_parameter<std::string>("controller_eip.ip", "127.0.0.1");
    nh_->declare_parameter<std::string>("controller_eip.host", "localhost");
    nh_->declare_parameter<double>("controller_eip.timeout", 3.0);

    // // Get parameters
    nh_->get_parameter("controller_eip.udp_port", params.port);
    nh_->get_parameter("controller_eip.ip",   params.ip);
    nh_->get_parameter("controller_eip.host", params.host);
    nh_->get_parameter("controller_eip.timeout", params.timeout);

    // Create sockets and connections
    socket    = shared_ptr<eip::socket::TCPSocket>(new eip::socket::TCPSocket(io_service));
    io_socket = shared_ptr<eip::socket::UDPSocket>(new eip::socket::UDPSocket(io_service, params.port, params.ip));
    session_  = std::make_unique<eip::Session>(socket, io_socket, nh_);

    return true;
}

// Connect
bool Controller_EIP::Connect(){
    try {
        session_->open(params.host);
        connected = true;
    } catch (...) {
        RCLCPP_WARN_ONCE(nh_->get_logger(),"Trying to connect, Will try to reconnect each %.2f seconds ...", params.timeout);
        connected = false;
    }
    return connected;
}

// Release
void Controller_EIP::Release(){
    //	Close the sockets
    session_->closeConnection(connection_num_);
    session_->close();
    connected = false;
}

// IsAvailable
bool Controller_EIP::Available() {
    return connected;
}

// Anybus methods
// Setters
// This function sets the value specified in the direction specified.
// Always the class identifier is the Assembly Object
// As is a set function, the instance_id is the 
void Controller_EIP::set(EIP_UINT addr, int channel, EIP_INT value) {
    switch (channel) {
        case 0:
        case 1:
            // Set normal value to the addr
            session_->setSingleAttribute(ASSEMBLY_CLASS, SET_INSTANCE_ID, addr, value);
        case 2:
            // Add 2 bytes to write to the buffer for channel 2
            session_->setSingleAttribute(ASSEMBLY_CLASS, SET_INSTANCE_ID, addr + 0x02, value);
        default:
            RCLCPP_ERROR(nh_->get_logger(), "The channel number has to be 1 or 2");
    }
} 

// Getters
// This function sets the value specified in the direction specified.
// Always the class identifier is the Assembly Object
// As is a get function, the instance_id is the 
EIP_INT Controller_EIP::get(EIP_UINT addr, int channel) {
    switch (channel) {
        case 0:
        case 1:
            return session_->getSingleAttribute(ASSEMBLY_CLASS, GET_INSTANCE_ID, addr, (EIP_UINT)0);
        case 2:
            return session_->getSingleAttribute(ASSEMBLY_CLASS, GET_INSTANCE_ID, addr + 0x02, (EIP_UINT)0);
        default:
            RCLCPP_ERROR(nh_->get_logger(), "The channel number has to be 1 or 2");
    }
}

} // namespace