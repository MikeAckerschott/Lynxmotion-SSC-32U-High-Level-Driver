/**
 * @file lowLevelServer.hpp
 * @brief Contains the LowLevelServer class that receives commands from the highLevelNode and handles them
*/

#ifndef LOWLEVELSERVER_HPP_
#define LOWLEVELSERVER_HPP_

#include "msg_srv/srv/single_servo_command.hpp"
#include "msg_srv/srv/multi_servo_command.hpp"
#include "msg_srv/srv/emergency_stop.hpp"
#include "msg_srv/srv/move_to_position.hpp"

#include "msg_srv/msg/move.hpp"
#include "msg_srv/msg/programmed_position.hpp"
#include "msg_srv/msg/servo_command.hpp"
#include "msg_srv/msg/state.hpp"

// include serial usb library
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <iostream>
#include <thread>
#include <vector>

class LowLevelServer : public rclcpp::Node
{
public:
    /**
    * @brief Constructor
    */
    LowLevelServer();

    /**
    * @brief Destructor
    */
    ~LowLevelServer();

    /**
    * @brief Handles client request for a single servo command and moves the selected servo if the request is valid
    * @param request The request from the client
    * @param response The response to send to the client
    */
    void handleSingleServoServiceRequest(
        const std::shared_ptr<msg_srv::srv::SingleServoCommand::Request> request,
        const std::shared_ptr<msg_srv::srv::SingleServoCommand::Response> response);

    /**
    * @brief Handles client request for a multi servo command and moves the selected servos if the request is valid
    * @param request The request from the client
    * @param response The response to send to the client
    */
    void handleMultiServoServiceRequest(
        const std::shared_ptr<msg_srv::srv::MultiServoCommand::Request> request,
        const std::shared_ptr<msg_srv::srv::MultiServoCommand::Response> response);

    /**
    * @brief Handles client request for an emergency stop and stops all servos
    * @param request The request from the client
    * @param response The response to send to the client
    */
    void handleEmergencyStop(
        const std::shared_ptr<msg_srv::srv::EmergencyStop::Request> request,
        const std::shared_ptr<msg_srv::srv::EmergencyStop::Response> response);

    /**
    * @brief Handles client request for a programmed position and moves the servos to the programmed position if the request is valid
    * @param request The request from the client
    * @param response The response to send to the client
    */
    void handleProgrammedPosition(
        const std::shared_ptr<msg_srv::srv::MoveToPosition::Request> request,
        const std::shared_ptr<msg_srv::srv::MoveToPosition::Response> response);

private:
    /**
    * @brief The service that receives a single servo command from the client. See highLevelNode.hpp for the message parameters
    */
    rclcpp::Service<msg_srv::srv::SingleServoCommand>::SharedPtr singleServoService;

    /**
    * @brief The service that receives a multi servo command from the client. See highLevelNode.hpp for the message parameters
    */
    rclcpp::Service<msg_srv::srv::MultiServoCommand>::SharedPtr multiServoService;

    /**
    * @brief The service that receives an emergency stop command from the client. See highLevelNode.hpp for the message parameters
    */
    rclcpp::Service<msg_srv::srv::EmergencyStop>::SharedPtr stopService;

    /**
    * @brief The service that receives a programmed position command from the client. See highLevelNode.hpp for the message parameters
    */
    rclcpp::Service<msg_srv::srv::MoveToPosition>::SharedPtr programmedPositionService;

    /**
    * @brief Used for creation of the serial port
    */
    boost::asio::io_service ioservice;

    /**
    * @brief The serial port to communicate with the robotic arm
    */
    boost::asio::serial_port serial_;
};

#endif /* LOWLEVELSERVER_HPP_ */