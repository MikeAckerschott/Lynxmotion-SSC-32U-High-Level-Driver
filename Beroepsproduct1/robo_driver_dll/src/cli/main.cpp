#include "rclcpp/rclcpp.hpp"
#include "commandParser.hpp"
#include "communicatorNode.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<CommunicatorNode> node =
      std::make_shared<CommunicatorNode>();

  CommandParser parser(node);

  std::cout<<"Welcome to the RoboDriver CLI. Check README.md for available commands"<<std::endl;

  bool exit_requested = false;

  while (rclcpp::ok() && !exit_requested)
  {
    std::cout<<"Enter command: "<<std::endl;
    // Check for user input
    std::string input;
    std::getline(std::cin, input);

    parser.parseCommand(input, node);
  }
  rclcpp::shutdown();
  return 0;
}