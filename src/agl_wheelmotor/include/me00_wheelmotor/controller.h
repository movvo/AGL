#ifndef CONTROLLER_MAIN_H
#define CONTROLLER_MAIN_H

#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string>

namespace roboteq {

class Controller {
    public:
        // Constructor
        Controller(std::shared_ptr<rclcpp::Node> &nh);
        virtual ~Controller();

        // Connection variables
        bool client_connected_;
        bool server_connected_;
        typedef struct {
            std::string ip;
            int port;
            double timeout;
        } tcpParams;
        typedef struct {
            int socket;
            int connectRes;
            sockaddr_in hint;
            char buffer[4096];
            std::string ip;
            int port;
            double timeout;
            tcpParams params;
        } ClientParams;
        ClientParams client;

        typedef struct {
            int socket;
            int client_socket;
            sockaddr_in hint;
            sockaddr_in client;
            socklen_t client_size = sizeof(client_socket);
            char host[NI_MAXHOST];      // Client's remote name
            char service[NI_MAXSERV];   // Service (i.e. port) the client is connect on
            char buffer[4096];
            tcpParams params;
        } ServerParams;
        ServerParams server;

        // Basic Methods
        bool Initialize();
        bool Connect();
        void Release();
        bool Available();
        std::string Send(std::string string);
        bool Read(std::string *string);

    private:
        std::shared_ptr<rclcpp::Node> nh_;

};

} // namespace

#endif // CONTROLLER_MAIN_H