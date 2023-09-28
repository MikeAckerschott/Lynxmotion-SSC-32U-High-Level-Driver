#include "lowLevelServer.hpp"
#include "singleServoCommand.hpp"
#include "multiServoCommand.hpp"
#include "commandUtils.hpp"
#include <boost/asio.hpp>

void LowLevelServer::singleServoRequest(SingleServoCommand command)
{
  command.sendCommand();
}

void LowLevelServer::multiServoRequest(MultiServoCommand command)
{
  command.sendCommand();
}

void LowLevelServer::stopCurrentMovement(boost::asio::serial_port &serial_)
{
  std::string commandAsString = "stop";
  commandAsString += Command::cr;

  Command command(commandAsString, serial_);
  command.sendCommand();
}

void LowLevelServer::handleProgrammedPosition(std::string position, boost::asio::serial_port &serial_)
{
  std::string commandAsString = "";
  if (position == "park")
  {
    commandAsString = CommandUtils::park;
    commandAsString += Command::cr;
  }
  else if (position == "ready")
  {
    commandAsString = CommandUtils::ready;
    commandAsString += Command::cr;
  }
  else if (position == "straight-up")
  {
    commandAsString = CommandUtils::straightUp;
    commandAsString += Command::cr;
  }
  else
  {
    return;
  }
  Command command(commandAsString, serial_);
  command.sendCommand();
}