
#include "me00_wheelmotor/controller.h"

namespace roboteq {

// Constructor
Controller::Controller(std::shared_ptr<rclcpp::Node> &nh) : nh_(nh) {
    Initialize();
}
Controller::~Controller() {
    Release();
}

// Initialize
bool Controller::Initialize() {
    // Declare parameters
    nh_->declare_parameter<std::string>("controller.ip", "192.168.1.0");
    nh_->declare_parameter<int>("controller.port", 5555);
    nh_->declare_parameter<double>("controller.timeout", 0.1);
    nh_->declare_parameter<std::string>("controller.listen_ip", "0.0.0.0");
    nh_->declare_parameter<int>("controller.listen_port", 8080);

    // Get parameters
    nh_->get_parameter("controller.ip", client.params.ip);
    nh_->get_parameter("controller.port", client.params.port);
    nh_->get_parameter("controller.timeout", client.params.timeout);
    nh_->get_parameter("controller.listen_ip", server.params.ip);
    nh_->get_parameter("controller.listen_port", server.params.port);

    return true;
}

// Connect
bool Controller::Connect(){
    while (!(client_connected_ && server_connected_) && rclcpp::ok()){
        try {
            if (!client_connected_) {
                // Init the Client server
                client.socket = socket(AF_INET, SOCK_STREAM, 0);
                if (client.socket == -1)
                    throw std::runtime_error("Connection to socket refused");

                client.hint.sin_family = AF_INET;
                client.hint.sin_port = htons(client.params.port);
                inet_pton(AF_INET, client.params.ip.c_str(), &client.hint.sin_addr);

                client.connectRes = connect(client.socket, (sockaddr*)&client.hint, sizeof(client.hint));
                if (client.connectRes == -1)
                    throw std::runtime_error("Connection to the address refused");
                client_connected_ = true;
                RCLCPP_INFO(nh_->get_logger(), "Client connected to server port: %d", client.params.port);
            }
            if (!server_connected_) {
                // Init the SERVER socket
                server.socket = socket(AF_INET, SOCK_STREAM, 0);
                if (server.socket == -1)
                    throw std::runtime_error("Connection to socket refused");
                server.hint.sin_family = AF_INET;
                server.hint.sin_port = htons(server.params.port);
                inet_pton(AF_INET, server.params.ip.c_str(), &server.hint.sin_addr);

                bind(server.socket, (sockaddr*)&server.hint, sizeof(server.hint));
                // Tell Winsock the socket is for listening
                listen(server.socket, SOMAXCONN);

                server.client_socket = accept(server.socket, (sockaddr*)&client, &server.client_size);
                
                memset(server.host, 0, NI_MAXHOST);
                memset(server.service, 0, NI_MAXSERV);

                if (getnameinfo((sockaddr*)&server.client, sizeof(server.client), server.host, NI_MAXHOST, server.service, NI_MAXSERV, 0) == 0){
                    RCLCPP_INFO(nh_->get_logger(), "%s connected on port %s", server.host, server.service);
                } else {
                    inet_ntop(AF_INET, &server.client.sin_addr, server.host, NI_MAXHOST);
                }
                // Close listening socket
                close(server.socket);
                memset(server.buffer, 0, 4096);
                server_connected_ = true;
                client_connected_ = true;
                RCLCPP_INFO(nh_->get_logger(), "Server connected to client port: %d", server.client_socket);
            }
        } 
        catch (std::runtime_error ex) {
            RCLCPP_WARN_ONCE(nh_->get_logger(),"Exception caught opening session: %s. Will try to reconnect each %.2f seconds ...", ex.what(), client.params.timeout);
            std::this_thread::sleep_for(std::chrono::seconds(uint32_t(client.params.timeout)));
            continue;
        }
    }
    RCLCPP_INFO(nh_->get_logger(), "Controller connected succesfully");
    return server_connected_ && client_connected_;
}

// Release
void Controller::Release(){
    //	Close the sockets
    close(client.socket);
    client_connected_ = false;
    close(server.socket);
    close(server.client_socket);
    server_connected_ = false;
}

// IsAvailable
bool Controller::Available() {
    return client_connected_ && server_connected_;
}

// Send
std::string Controller::Send(std::string str) {
    try {
        if (client_connected_) {
            // Send the string command via socket
            int sendRes = send(client.socket, str.c_str(), str.size() + 1, 0);
            if (sendRes == -1)
                throw std::runtime_error("Error sending");

            // Wait for response
            memset(client.buffer, 0, 4096);
            int bytesReceived = recv(client.socket, client.buffer, 4096, 0);
            if (bytesReceived == -1) {
                throw std::runtime_error("There was an error getting response from server");
            } else {
                //	Return response
                return std::string(client.buffer, bytesReceived);
            }
        }
    } catch (std::runtime_error ex) {
        RCLCPP_ERROR(nh_->get_logger(), "Error sending: %s", ex.what());
        return "";
    }
    return "";
}

// Read
bool Controller::Read(std::string* str){
    try {
        if(server_connected_) {
            // Read from the socket
            memset(server.buffer, 0, 4096);

            // Wait for client to send data
            int bytesReceived = recv(server.client_socket, server.buffer, 4096, 0);
            if (bytesReceived == -1) {
                RCLCPP_INFO(nh_->get_logger(), "Error in reciveing");
                return false;
            }
    
            if (bytesReceived == 0) {
                RCLCPP_INFO(nh_->get_logger(), "Client disconnected");
                server_connected_ = false;
                return false;
            }
            *str = std::string(server.buffer, 0, bytesReceived);

            //Echo message back to client
            send(server.client_socket, server.buffer, bytesReceived+1, 0);
        }
    } catch (std::string ex) {
        RCLCPP_ERROR(nh_->get_logger(), "Error reading: " + ex);
        return false;
    }
    return true;
}

} // namespace