/**
 * @file commandUtils.hpp
 * @brief Contains the CommandUtils class that provides helpful variables which make sending commands to the robotic arm easier 
*/

#ifndef COMMANDUTILS_HPP_
#define COMMANDUTILS_HPP_

#include <iostream>

class CommandUtils {
public:
  /**
  * @brief The carraiage return character
  */
  static char cr;

  /**
  * @brief Command to send over the serial port to stop robotic arm movement
  */
  static std::string cancel;

  /**
  * @brief Command to send over the serial port to move the robotic arm to the park position
  */
  static std::string park;

  /**
  * @brief Command to send over the serial port to move the robotic arm to the ready position
  */
  static std::string ready;

  /**
  * @brief Command to send over the serial port to move the robotic arm to the straight-up position
  */
  static std::string straightUp;
};

#endif /* COMMANDUTILS_HPP_ */