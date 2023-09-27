#include "rclcpp/rclcpp.hpp"

#include "highLevelNode.hpp"

#include "msg_srv/srv/single_servo_command.hpp"

// TODO remove after testing
#include "../lld/command.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  // rclcpp::init(argc, argv);

  // std::shared_ptr<HighLevelNode> highLevelServer = std::make_shared<HighLevelNode>();

  // rclcpp::spin(highLevelServer);

  // rclcpp::shutdown();
  boost::asio::io_service ioservice;
  boost::asio::serial_port serial_(ioservice, "/dev/ttyUSB0");

  // rclcpp::TimerBase::SharedPtr timer_;

  std::string commandString = "# 0 P1500 # 1 P1450 # 2 P700 # 3 P1450 # 4 P944 # 5 P1450 T3000";
  commandString += Command::cr;

  Command moveCommand(commandString, serial_);
  moveCommand.sendCommand();

  commandString = "Q";
  commandString += Command::cr;
  Command command(commandString, serial_);

  char temp;

  while (temp != '.')
  {
    command.sendCommand();
    temp = command.readMostRecentChar();
    std::cout << "read: " << temp << std::endl;
  }

  return 0;
}
