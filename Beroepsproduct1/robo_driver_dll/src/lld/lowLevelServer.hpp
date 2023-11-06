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

class LowLevelServer
{
public:

    /**
     * @brief Sends a singleServoRequest to the robotic arm
     * @param command The command that needs to be send to the robotic arm
     */
    static void singleServoRequest(SingleServoCommand command);

    /**
     * @brief Sends a multiServoCommand to the robotic arm
     * @param command The command that needs to be send to the robotic arm 
     */
    static void multiServoRequest(MultiServoCommand command);

    /**
     * @brief Stops the current movement of the robotic arm
     * @param serial_ The serial port that is used to communicate with the robotic arm
     */
    static void stopCurrentMovement(boost::asio::serial_port &serial_);

    /**
     * @brief Moves the robotic arm to the given position
     * @param position The position the robotic arm needs to move to
     * @param serial_ The serial port that is used to communicate with the robotic arm
     */
    static void handleProgrammedPosition(std::string position, boost::asio::serial_port &serial_);
};

#endif /* LOWLEVELSERVER_HPP_ */