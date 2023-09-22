#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

// custom msg types
#include "msg_srv/msg/move.hpp"
#include "msg_srv/msg/programmed_position.hpp"
#include "msg_srv/msg/servo_command.hpp"
#include "msg_srv/msg/state.hpp"

#include "command.hpp"
#include "commandUtils.hpp"
#include "multiServoCommand.hpp"
#include "singleServoCommand.hpp"

#include "lowLevelServer.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // boost::asio::io_service ioservice;

  std::cout << "creating lowLevelServer" << std::endl;
  std::shared_ptr<LowLevelServer> lowLevelServer =
      std::make_shared<LowLevelServer>();

  std::cout << "done creating lowLevelServer" << std::endl;
  std::cout << "spinning lowLevelServer" << std::endl;
  rclcpp::spin(lowLevelServer);

  rclcpp::shutdown();
  return 0;
}
