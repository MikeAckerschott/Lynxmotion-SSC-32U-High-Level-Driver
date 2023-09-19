#include "highLevelNode.hpp"
#include <cstdint>

using namespace std::chrono_literals;

HighLevelNode::HighLevelNode() : Node("high_level_client")
{
  singleServoClient_ = create_client<msg_srv::srv::SingleServoCommand>("single_servo_command");

  multiServoClient_ = create_client<msg_srv::srv::MultiServoCommand>("multi_servo_command");

  stopClient_ = create_client<msg_srv::srv::EmergencyStop>("emergency_stop");

  programmedPositionClient_ = create_client<msg_srv::srv::MoveToPosition>("programmed_position");

  while (!singleServoClient_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  RCLCPP_INFO(get_logger(), "Service detected!");
}

HighLevelNode::~HighLevelNode()
{
}

void HighLevelNode::sendSingleServoCommand(short servo, unsigned long long angle, unsigned long long speed, unsigned long long time)
{
  // std::shared_ptr<msg_srv::srv::SingleServoCommand::Request> request_;
  auto request_ = std::make_shared<msg_srv::srv::SingleServoCommand::Request>();

  request_->position.target_servo = servo; // Set the target_servo field to 1
  request_->position.position = angle;     // Set the position field to 1000
  request_->position.duration = time;      // Set the duration field to 100
  request_->position.speed = speed;

  std::cout << "sending request: " << (short)request_->position.target_servo << " - " << request_->position.position << " - " << request_->position.duration << " - " << request_->position.speed << std::endl;
  auto result = singleServoClient_->async_send_request(request_);
  std::cout << "request sent" << std::endl;
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(get_logger(), "Sheesh %x Wazza", result.get()->finished);
  }
  else
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to call service ");
  }
}

void HighLevelNode::sendMultiServoCommand(std::vector<std::vector<long long>> arguments)
{
  std::cout << "test1" << std::endl;
  // std::shared_ptr<msg_srv::srv::MultiServoCommand::Request> request_;
  auto request_ = std::make_shared<msg_srv::srv::MultiServoCommand::Request>();

  std::cout << "test2" << std::endl;
  msg_srv::msg::Move move;
  // std::cout << request_->positions.instruction.size() << std::endl;
  for (int i = 0; i < arguments.size(); ++i)
  {
    std::cout << "arguments: " << arguments.at(i).at(0) << " " << arguments.at(i).at(1) << " " << arguments.at(i).at(2) << " " << arguments.at(i).at(3) << std::endl;
    msg_srv::msg::ServoCommand servoCommand;
    servoCommand.target_servo = arguments.at(i).at(0);
    servoCommand.position = arguments.at(i).at(1);
    servoCommand.speed = arguments.at(i).at(2);
    servoCommand.duration = arguments.at(i).at(3);
    move.instruction.push_back(servoCommand);
    std::cout << "test3" << std::endl;
  }

  request_->positions = move;
  std::cout << "test4" << std::endl;

  auto result = multiServoClient_->async_send_request(request_);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {

    if (result.get()->finished)
    {
      // RCLCPP_INFO(get_logger(), "Sheesh %x Wazza", result.get()->finished);
      RCLCPP_INFO(get_logger(), "Command received succesfully! Moving robotic arm...");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Usage of incorrect syntax. Please try again.");
    }
  }
  else
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to call service");
  }
}

void HighLevelNode::sendStopCommand()
{
  // std::shared_ptr<msg_srv::srv::EmergencyStop::Request> request_;
  auto request_ = std::make_shared<msg_srv::srv::EmergencyStop::Request>();

  auto result = stopClient_->async_send_request(request_);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(get_logger(), "Sheesh %x Wazza", result.get()->stopped);
  }
  else
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to call service");
  }
}

void HighLevelNode::sendProgrammedPositionCommand(std::string programmedPosition)
{
  std::cout << "ProgrammedPosition: " << programmedPosition << std::endl;
  if (programmedPosition != "park" && programmedPosition != "ready" && programmedPosition != "straight-up")
  {
    std::cout << "ProgrammedPosition not recognized" << std::endl;
    return;
  }

  // std::shared_ptr<msg_srv::srv::MoveToPosition::Request> request_;
  auto request_ = std::make_shared<msg_srv::srv::MoveToPosition::Request>();
  msg_srv::msg::ProgrammedPosition programmedPositionMsg;
  programmedPositionMsg.programmed_position = programmedPosition;
  request_->position = programmedPositionMsg;

  auto result = programmedPositionClient_->async_send_request(request_);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(get_logger(), "Sheesh %x Wazza", result.get()->finished);
  }
  else
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to call service");
  }
}
