/**
 * @file commandParser.hpp
 * @brief Parses the commands sent from the CLI and sends them to the low level driver
 */

#ifndef COMMANDPARSER_HPP_
#define COMMANDPARSER_HPP_

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>

#include "msg_srv/srv/single_servo_command.hpp"
#include "msg_srv/srv/multi_servo_command.hpp"

#include "communicatorNode.hpp"

class CommandParser
{

public:
  /**
   * @brief Constructor
   */
  CommandParser();

  /**
   * @brief Destructor
   */
  ~CommandParser();

  /**
   * @brief Parses the command and sends correct command to the low level driver
   * @param command The command to parse (string from CLI)
   * @param node The node that sends the command to the low level driver
   */
  void parseCommand(std::string command, std::shared_ptr<CommunicatorNode> node);

  /**
   * @brief Parses the command and sends singleServoCommand on successful parsing
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @param command The command to parse (the entire string from CLI)
   * @param node The node that sends the command to the low level driver
   * @return True if the command was parsed correctly, false if not
   */
  bool parseSingleServoCommand(std::string commandType, std::string command, std::shared_ptr<CommunicatorNode> node);

  /**
   * @brief Parses the command and sends multiServoCommand on successful parsing
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @param command The command to parse (the entire string from CLI)
   * @param node The node that sends the command to the low level driver
   * @return True if the command was parsed correctly, false if not
   */
  bool parseMultiServoCommand(std::string commandType, std::string command, std::shared_ptr<CommunicatorNode> node);

  /**
   * @brief Parses the command and sends stopCommand on successful parsing
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @param command The command to parse (the entire string from CLI)
   * @param node The node that sends the command to the low level driver
   * @return True if the command was parsed correctly, false if not
   */
  bool parseStopCommand(std::string commandType, std::string command, std::shared_ptr<CommunicatorNode> node);

  /**
   * @brief Parses the command and sends programmedPositionCommand on successful parsing
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @param command The command to parse (the entire string from CLI)
   * @param node The node that sends the command to the low level driver
   * @return True if the command was parsed correctly, false if not
   */
  bool parseProgrammedPositionCommand(std::string commandType, std::string command, std::shared_ptr<CommunicatorNode> node);

private:
  bool isNumber(const std::string &s);
};

#endif // COMMANDPARSER_HPP_