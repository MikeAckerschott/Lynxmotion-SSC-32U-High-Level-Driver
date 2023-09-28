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

#include "singleServoCommand.hpp"
#include "multiServoCommand.hpp"

// include serial usb library
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <iostream>
#include <thread>
#include <vector>

/**
 * @class LowLevelServer
 * @brief Dynamically linked libray used by the HLD that implements the basic functionality of the robotic arm
 */

class LowLevelServer : public rclcpp::Node
{
public:

    /**
     * @brief Handles client request for a single servo command and moves the selected servo if the request is valid
     * @param request The request from the client
     * @param response The response to send to the client
     */
    static void singleServoRequest(SingleServoCommand command);

    /**
     * @brief Handles client request for a multi servo command and moves the selected servos if the request is valid
     * @param request The request from the client
     * @param response The response to send to the client
     */
    static void multiServoRequest(MultiServoCommand command);

    /**
     * @brief Handles client request for an emergency stop and stops all servos
     * @param request The request from the client
     * @param response The response to send to the client
     */
    static void stopCurrentMovement(boost::asio::serial_port &serial_);

    /**
     * @brief Handles client request for a programmed position and moves the servos to the programmed position if the request is valid
     * @param request The request from the client
     * @param response The response to send to the client
     */
    static void handleProgrammedPosition(std::string position, boost::asio::serial_port &serial_);
};

#endif /* LOWLEVELSERVER_HPP_ */