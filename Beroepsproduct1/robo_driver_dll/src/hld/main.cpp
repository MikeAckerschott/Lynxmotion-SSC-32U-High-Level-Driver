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
  rclcpp::init(argc, argv);

  std::shared_ptr<HighLevelNode> highLevelServer = std::make_shared<HighLevelNode>();

  rclcpp::spin(highLevelServer);

  rclcpp::shutdown();

  return 0;
}
