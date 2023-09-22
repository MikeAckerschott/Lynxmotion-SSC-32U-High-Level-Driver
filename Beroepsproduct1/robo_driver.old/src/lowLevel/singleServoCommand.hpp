#ifndef SINGLESERVOCOMMAND_HPP_
#define SINGLESERVOCOMMAND_HPP_

#include "command.hpp"
#include <boost/asio.hpp>
#include <chrono>
#include <thread>

class SingleServoCommand : public Command {
public:
  enum movementType {
    MOVE_WITH_TIME = 0,
    MOVE_WITH_SPEED = 1,
    NO_TYPE = 2,
  };

  SingleServoCommand(short pin, short desiredPulseWidth, long movement,
                     movementType type, boost::asio::serial_port &serial);

  ~SingleServoCommand();
  std::string getCommandString(short pin, short desiredPulseWidth,
                               long movement, movementType type);
  std::string getCommandString();

  short
      pin; // pin / channel to which the servo is connected (0 to 31) in decimal
  short desiredPulseWidth; // desired pulse width (normally 500 to 2500) in
                           // microseconds
  long movement;
  movementType type;
};

#endif /* SINGLESERVOCOMMAND_HPP_ */