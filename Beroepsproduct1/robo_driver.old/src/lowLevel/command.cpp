#include "command.hpp"

Command::Command(std::string command, boost::asio::serial_port &serial)
    : command(command), serial(serial) {}

Command::~Command() {}

void Command::sendCommand() {
  std::cout << "sending command: " << std::endl << command << std::endl;

  boost::asio::streambuf b;
  std::ostream os(&b);
  os << command;
  boost::asio::write(serial, b.data());
  os.flush();

}