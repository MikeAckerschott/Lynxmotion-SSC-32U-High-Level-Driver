#include "singleServoCommand.hpp"
#include "command.hpp"

SingleServoCommand::SingleServoCommand(short pin, short desiredPulseWidth,
                                       long movement, movementType type,
                                       boost::asio::serial_port &serial)
    : pin(pin), desiredPulseWidth(desiredPulseWidth),
      movement(movement), type(type),
      Command(getCommandString(pin, desiredPulseWidth, movement, type), serial)
{

  std::cout << "pin: " << pin << " desiredPulseWidth: " << desiredPulseWidth << " movement: " << movement << " type: " << type << std::endl;
  if (desiredPulseWidth > 2500 || desiredPulseWidth < 500)
  {
    throw std::invalid_argument(
        "desiredPulseWidth must be between 500 and 2500");
  }
}

SingleServoCommand::~SingleServoCommand() {}

std::string SingleServoCommand::getCommandString(short pin,
                                                 short desiredPulseWidth,
                                                 long movement,
                                                 movementType type)
{
  std::string commandString =
      "# " + std::to_string(pin) + " P" + std::to_string(desiredPulseWidth);

  if (type == MOVE_WITH_TIME)
  {
    commandString += " T" + std::to_string(movement);
  }
  else if (type == MOVE_WITH_SPEED)
  {
    commandString += " S" + std::to_string(movement);
  }
  commandString += " ";
  commandString += '\r';

  return commandString;
}

std::string SingleServoCommand::getCommandString()
{
  std::string commandString =
      "# " + std::to_string(pin) + " P" + std::to_string(desiredPulseWidth);

  if (this->type == MOVE_WITH_TIME)
  {
    commandString += " T" + std::to_string(movement);
  }
  else if (this->type == MOVE_WITH_SPEED)
  {
    commandString += " S" + std::to_string(movement);
  }
  commandString += " ";
  commandString += '\r';

  return commandString;
}