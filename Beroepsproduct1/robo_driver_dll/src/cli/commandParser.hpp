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

/**
 * @class CommandParser
 * @brief Parses the commands sent from the CLI and sends them to the low level driver
 * @details Currently supports commands: singleServo, multiServo, stop, programmedPosition
 */

class CommandParser
{

public:
  /**
   * @brief Constructor
   * @param node The node that sends the command to the low level driver. also used for logging with RCLCPP_INFO
   */
  CommandParser(std::shared_ptr<CommunicatorNode> node);

  /**
   * @brief Destructor
   */
  ~CommandParser();

  /**
   * @brief Parses the command and sends correct command to the low level driver
   * @param command The command to parse (string from CLI)
   */
  void parseCommand(std::string command);

  /**
   * @brief Parses the command and sends singleServoCommand on successful parsing
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @param command The command to parse (the entire string from CLI)
   * @return True if the command was parsed correctly, false if not
   */
  bool parseSingleServoCommand(std::string commandType, std::string command);

  /**
   * @brief Parses the command and sends multiServoCommand on successful parsing
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @param command The command to parse (the entire string from CLI)
   * @return True if the command was parsed correctly, false if not
   */
  bool parseMultiServoCommand(std::string commandType, std::string command);

  /**
   * @brief Parses the command and sends stopCommand on successful parsing. 
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @param command The command to parse (the entire string from CLI)
   * @return True if the command was parsed correctly, false if not
   */
  bool parseStopCommand(std::string commandType);

  /**
   * @brief Parses the command and sends startCommand on successful parsing. starting means that the HLD will exit the emergency stop state
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @return True if the command was parsed correctly, false if not
  */

  bool parseStartCommand(std::string commandType);

  /**
   * @brief Parses the command and sends programmedPositionCommand on successful parsing
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @param command The command to parse (the entire string from CLI)
   * @return True if the command was parsed correctly, false if not
   */
  bool parseProgrammedPositionCommand(std::string commandType, std::string command);

  /**
   * @brief Parses the command and sends skipCommand on successful parsing. skipping means that the HLD will skip the current movement and executes the next one in queue
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @return True if the command was parsed correctly, false if not
   */
  bool parseSkipCommand(std::string commandType);

  /**
   * @brief Parses the command and sends emptyQueueCommand on successful parsing. emptying means that the HLD will empty the queue of movements and stop the current movement
   * @param commandType The type of command to parse (typically the first word sent from CLI)
   * @return True if the command was parsed correctly, false if not
   */

  bool parseEmptyQueueCommand(std::string commandType);

private:

  /**
   * @brief Checks if the string is a number
   * @param s The string to check
   * @return True if the string is a number, false if not
   */
  bool isNumber(const std::string &s);

  /**
   * @brief The node that sends the command to the low level driver. also used for logging with RCLCPP_INFO
   */

  std::shared_ptr<CommunicatorNode> communicatorNode_;

  /**
   * @brief fills the reference variables with the correct values from the commandArguments string
   * @param s The string to check
   * @param servoNumber The servo number to fill
   * @param angle The angle to fill
   * @param movement The movement to fill
   * @param movementType The movement type to fill 
   * @return True if the string is correclty formatted and the integer values are filled, false if not
   */
   
  bool getSingleServoCommandArguments(std::string commandArguments, int &servoNumber, int &angle, int &movement, std::string &movementType);
};

#endif // COMMANDPARSER_HPP_