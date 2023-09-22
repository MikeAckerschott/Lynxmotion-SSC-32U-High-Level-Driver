#include "multiServoCommand.hpp"
#include "command.hpp"
#include "singleServoCommand.hpp"

MultiServoCommand::MultiServoCommand(std::vector<SingleServoCommand> commands,
                                     boost::asio::serial_port &serial)
    : commands(commands), Command(getCommandString(commands), serial) {}

MultiServoCommand::~MultiServoCommand() {}

std::string MultiServoCommand::getCommandString(std::vector<SingleServoCommand> commands) {
  // speed or time can be NULL
  std::string command = "";
  for (int i = 0; i < commands.size(); i++) {
    command += commands[i].getCommandString();
    command.pop_back();
  }
  command += '\r';
  return command;
}

std::string MultiServoCommand::getCommandString() {
  // speed or time can be NULL
  std::string command = "";
  for (int i = 0; i < commands.size(); i++) {
    command += commands[i].getCommandString();
    command.pop_back();
  }
  command += '\r';
  return command;
}