/**
 * @file command.hpp
 * @brief Contains the Command class that defines the syntax for a command to be sent over the serial port
 */

#ifndef COMMAND_HPP_
#define COMMAND_HPP_

#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>

/**
 * @class Command
 * @brief Defines the syntax for a command to be sent over the serial port
 */

class Command
{
public:
  /**
   * @brief Constructor
   * @param command The command to send over the serial port
   * @param serial The serial port to send the command over
   */
  Command(std::string command, boost::asio::serial_port &serial);

  char readMostRecentChar();

  /**
   * @brief Destructor
   */
  virtual ~Command();

  /**
   * @brief Sends the command over the serial port
   */
  virtual void sendCommand();

  /**
   * @brief The carriage return character
   */
  static const char cr = 13;

private:
  /**
   * @brief command to send over the serial port
   */
  std::string command;

  /**
   * @brief serial port to send the command over
   */
  boost::asio::serial_port &serial;
};

#endif /* COMMAND_HPP_ */