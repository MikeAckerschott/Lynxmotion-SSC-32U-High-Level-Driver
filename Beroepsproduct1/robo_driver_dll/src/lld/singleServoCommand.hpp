#ifndef SINGLESERVOCOMMAND_HPP_
#define SINGLESERVOCOMMAND_HPP_

/**
 * @file singleServoCommand.hpp
 * @brief Contains the SingleServoCommand class that defines the syntax for a single servo command to be sent over the serial port
 * @details This class is used to send a command to a single servo over the serial port
 * @see Command
*/

#include "command.hpp"
#include <boost/asio.hpp>
#include <chrono>
#include <thread>

/**
 * @class SingleServoCommand
 * @brief Defines the syntax for a single servo command to be sent over the serial port
 * @details This class is used to send a command to a single servo over the serial port
 * @see Command
*/

class SingleServoCommand : public Command {
public:

/**
 * @brief Defines the type of movement of the servo
 * @details This enum is used to define the type of movement of the servo
 * @param MOVE_WITH_TIME The servo moves with a duration
 * @param MOVE_WITH_SPEED The servo moves with a speed
 * @param NO_TYPE The servo does not mvoe with a type. used for multiservocommands where the type is already defined from another servo, since a type does not have to be defined for every servo
*/
  enum movementType {
    MOVE_WITH_TIME = 0,
    MOVE_WITH_SPEED = 1,
    NO_TYPE = 2,
  };

    /**
   * @brief Constructor
   * @param pin The pin of the servo
   * @param desiredPulseWidth The desired pulse width of the servo
   * @param movement The movement of the servo
   * @param type The type of movement of the servo (MOVE_WITH_TIME, MOVE_WITH_SPEED, NO_TYPE)
   * @param serial The serial port to send the command over
   */

  SingleServoCommand(short pin, short desiredPulseWidth, long movement, movementType type, boost::asio::serial_port &serial);

  /**
   * @brief Destructor
  */
  ~SingleServoCommand();

  /**
   * @brief Gets the command string from the parameters. Used in constructor, since member variables are not initialized yet
   * @param pin The pin of the servo
   * @param desiredPulseWidth The desired pulse width of the servo
   * @param movement The movement of the servo
   * @param type The type of movement of the servo (MOVE_WITH_TIME, MOVE_WITH_SPEED, NO_TYPE)
   * @return The command string that can be sent over the serial port
  */
  std::string getCommandString(short pin, short desiredPulseWidth, long movement, movementType type);

  /**
   * @brief Gets the command string from the member variables
   * @return The command string that can be sent over the serial port
  */
  std::string getCommandString();

private:
/**
 * @brief The pin of the servo
*/
  short pin; 

/**
 * @brief The desired pulse width of the servo
 */
  short desiredPulseWidth; 

/**
 * @brief The movement of the servo
 */
  long movement;

/**
 * @brief The type of movement of the servo (MOVE_WITH_TIME, MOVE_WITH_SPEED, NO_TYPE)
 */
  movementType type;
};

#endif /* SINGLESERVOCOMMAND_HPP_ */