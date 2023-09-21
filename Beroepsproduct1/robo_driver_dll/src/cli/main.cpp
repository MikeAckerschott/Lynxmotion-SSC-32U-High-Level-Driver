#include "rclcpp/rclcpp.hpp"
#include "commandParser.hpp"
#include "communicatorNode.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  CommandParser parser;

  std::shared_ptr<CommunicatorNode> node =
      std::make_shared<CommunicatorNode>();

  bool exit_requested = false;

  while (rclcpp::ok() && !exit_requested)
  {
    // Check for user input
    std::cout << "Enter 'send' followed by 4 integers (e.g., 'send 0 45 0 "
                 "5000') or 'exit' to exit: ";
    std::string input;
    std::getline(std::cin, input);

    parser.parseCommand(input, node);
  }
  rclcpp::shutdown();
  return 0;
}