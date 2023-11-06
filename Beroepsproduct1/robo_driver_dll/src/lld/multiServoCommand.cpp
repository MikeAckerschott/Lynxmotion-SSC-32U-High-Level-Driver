#include "multiServoCommand.hpp"
#include "command.hpp"
#include "singleServoCommand.hpp"

MultiServoCommand::MultiServoCommand(std::vector<SingleServoCommand> commands,
                                     boost::asio::serial_port &serial)
    : Command(getCommandString(commands), serial), commands(commands) {}

MultiServoCommand::~MultiServoCommand() {}

std::string MultiServoCommand::getCommandString(std::vector<SingleServoCommand> commands)
{
  // speed or time can be NULL
  std::string command = "";
  for (long unsigned int i = 0; i < commands.size(); i++)
  {
    command += commands[i].getCommandString();
    command.pop_back();
  }
  command += '\r';
  return command;
}

std::string MultiServoCommand::getCommandString()
{
  // speed or time can be NULL
  std::string command = "";
  for (long unsigned int i = 0; i < commands.size(); i++)
  {
    command += commands[i].getCommandString();
    command.pop_back();
  }
  command += '\r';
  return command;
}