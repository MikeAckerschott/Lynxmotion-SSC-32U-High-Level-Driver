#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "highLevelNode.hpp"

#include "msg_srv/srv/single_servo_command.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>

#include "commandParser.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  CommandParser parser;

  std::shared_ptr<HighLevelNode> highLevelNode2 =
      std::make_shared<HighLevelNode>();

  bool exit_requested = false;

  while (rclcpp::ok() && !exit_requested)
  {
    // Check for user input
    std::cout << "Enter 'send' followed by 4 integers (e.g., 'send 0 45 0 "
                 "5000') or 'exit' to exit: ";
    std::string input;
    std::getline(std::cin, input);

    parser.parseCommand(input, highLevelNode2);
  }
  rclcpp::shutdown();
  return 0;
}
