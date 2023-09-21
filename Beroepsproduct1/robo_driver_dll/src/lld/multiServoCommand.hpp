/**
 * @file multiServoCommand.hpp
 * @brief Contains the MultiServoCommand class that defines the syntax for a multiple servo command to be sent over the serial port
*/

#ifndef MULTISERVOCOMMAND_HPP_
#define MULTISERVOCOMMAND_HPP_

#include "command.hpp"
#include <boost/asio.hpp>
#include <chrono>
#include <thread>
#include "singleServoCommand.hpp"
#include <vector>

class MultiServoCommand : public Command
{
public:
  /**
   * @brief Constructor
   * @param commands The commands to send over the serial port, syntaxed as an array of multiple SingleServoCommands
   * @param serial The serial port to send the command over
   */
  MultiServoCommand(std::vector<SingleServoCommand> commands, boost::asio::serial_port &serial);

  /**
   * @brief Destructor
   */
  ~MultiServoCommand();

  /**
   * @brief Gets the command string from the vector of singleServoCommands
   * @param commands The commands to send over the serial port, syntaxed as an array of multiple SingleServoCommands
   * @return The command string that can be sent over the serial port
   */
  std::string getCommandString(std::vector<SingleServoCommand> commands);

  /**
   * @brief Gets the command string from the vector of singleServoCommands
   * @return The command string that can be sent over the serial port
   */
  std::string getCommandString();

private:
  /**
   * @brief vector of singleServoCommands that form a multiServoCommand
   */
  std::vector<SingleServoCommand> commands;
};

#endif /* MULTISERVOCOMMAND_HPP_ */