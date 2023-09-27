#include "rclcpp/rclcpp.hpp"
#include "commandParser.hpp"
#include "communicatorNode.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);


  std::shared_ptr<CommunicatorNode> node =
      std::make_shared<CommunicatorNode>();

        CommandParser parser(node);

  bool exit_requested = false;

  while (rclcpp::ok() && !exit_requested)
  {
    // Check for user input
    std::cout << "Enter command: ";
    std::string input;
    std::getline(std::cin, input);

    parser.parseCommand(input, node);
  }
  rclcpp::shutdown();
  return 0;
}