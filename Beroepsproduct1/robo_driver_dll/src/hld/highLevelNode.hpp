/**
 * @file highLevelNode.hpp
 * @brief Contains the HighLevelNode class that sends commands to the low level driver based on the input from the CLI
 */

#ifndef HIGH_LEVEL_NODE_HPP_
#define HIGH_LEVEL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "msg_srv/srv/single_servo_command.hpp"
#include "msg_srv/srv/multi_servo_command.hpp"
#include "msg_srv/srv/emergency_stop.hpp"
#include "msg_srv/srv/move_to_position.hpp"

#include "msg_srv/msg/move.hpp"
#include "msg_srv/msg/servo_command.hpp"
#include "msg_srv/msg/programmed_position.hpp"

#include "../lld/command.hpp"
#include "../lld/singleServoCommand.hpp"
#include "../lld/multiServoCommand.hpp"
#include "../lld/lowLevelServer.hpp"
#include "../lld/commandUtils.hpp"

#include "servoUtils.hpp"
#include "context.hpp"
#include "idleState.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>

// include library for makeshare

/**
 * @class HighLevelNode
 * @brief Sends commands to robotic arm based on input from the CLI. Uses the low level driver to send commands to the robotic arm
 */

class Context;

class HighLevelNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  HighLevelNode();

  /**
   * @brief Destructor
   */
  ~HighLevelNode();

  /**
   * @brief Handles client request for a single servo command and moves the selected servo if the request is valid
   * @param request The request from the client
   * @param response The response to send to the client
   * @return void
   */

  void handleSingleServoServiceRequest(
      const std::shared_ptr<msg_srv::srv::SingleServoCommand::Request> request,
      const std::shared_ptr<msg_srv::srv::SingleServoCommand::Response> response);

  /**
   * @brief Handles client request for a multi servo command and moves the selected servos if the request is valid
   * @param request The request from the client
   * @param response The response to send to the client
   * @return void
   */
  void handleMultiServoServiceRequest(
      const std::shared_ptr<msg_srv::srv::MultiServoCommand::Request> request,
      const std::shared_ptr<msg_srv::srv::MultiServoCommand::Response> response);

  /**
   * @brief Handles client request for an emergency stop and stops all servos
   * @param request The request from the client
   * @param response The response to send to the client
   * @return void
   */
  void handleEmergencyStop(
      const std::shared_ptr<msg_srv::srv::EmergencyStop::Request> request,
      const std::shared_ptr<msg_srv::srv::EmergencyStop::Response> response);

  /**
   * @brief Handles client request for a programmed position and moves the servos to the programmed position if the request is valid
   * @param request The request from the client
   * @param response The response to send to the client
   * @return void
   */
  void handleProgrammedPosition(
      const std::shared_ptr<msg_srv::srv::MoveToPosition::Request> request,
      const std::shared_ptr<msg_srv::srv::MoveToPosition::Response> response);

private:
  /**
   * @brief The service that receives a single servo command from the client.
   */
  rclcpp::Service<msg_srv::srv::SingleServoCommand>::SharedPtr singleServoService;

  /**
   * @brief The service that receives a multi servo command from the client.
   */
  rclcpp::Service<msg_srv::srv::MultiServoCommand>::SharedPtr multiServoService;

  /**
   * @brief The service that receives an emergency stop command from the client.
   */
  rclcpp::Service<msg_srv::srv::EmergencyStop>::SharedPtr stopService;

  /**
   * @brief The service that receives a programmed position command from the client.
   */
  rclcpp::Service<msg_srv::srv::MoveToPosition>::SharedPtr programmedPositionService;

  Context *context;

private:
  /**
   * @brief Used for creation of the serial port
   */
  boost::asio::io_service ioservice;

  /**
   * @brief The serial port to communicate with the robotic arm
   */
  boost::asio::serial_port serial_;

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // HIGH_LEVEL_NODE_HPP_
