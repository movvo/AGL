#ifndef CONTROLLER_EIP_MAIN_H
#define CONTROLLER_EIP_MAIN_H

#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string>

// ODVA EtherNet/IP
#include "odva_ethernetip/session.h"
#include "odva_ethernetip/socket/socket.h"
#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"

using std::vector;
using boost::shared_ptr;
using eip::Session;
using eip::socket::Socket;

namespace roboteq {

class Controller_EIP {
    public:
        Controller_EIP(std::shared_ptr<rclcpp::Node> &nh);
        ~Controller_EIP();

        // Session
        boost::asio::io_service io_service;
	    shared_ptr<eip::socket::TCPSocket> socket;
	    shared_ptr<eip::socket::UDPSocket> io_socket;
        std::unique_ptr<eip::Session> session_;

        // Methods
        bool Initialize();
        bool Connect();
        void Release(); 
        bool Available();

        // EtherNet/IP fucntions to set/get
        // values of the buffer in the Anybus.
        void set(EIP_UINT addr, int channel, EIP_INT value);
        EIP_INT get(EIP_UINT addr, int channel);
    private:
        // NodeHandle
        std::shared_ptr<rclcpp::Node> nh_;
        // data for sending to Anybus to keep UDP session alive
        int connection_num_;
        EIP_UDINT sequence_num_;
        
        // Parameters
        bool connected;
        typedef struct {
            int port;
            std::string ip;
            std::string host;
            double timeout;
        } Parameters;
        Parameters params;
};

} // namespace

#endif // CONTROLLER_EIP_MAIN_H