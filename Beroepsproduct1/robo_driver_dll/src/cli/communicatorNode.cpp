#include "communicatorNode.hpp"

CommunicatorNode::CommunicatorNode() : Node("communicator_node")
{
    singleServoClient_ = create_client<msg_srv::srv::SingleServoCommand>("single_servo_command");

    multiServoClient_ = create_client<msg_srv::srv::MultiServoCommand>("multi_servo_command");

    stopClient_ = create_client<msg_srv::srv::EmergencyStop>("emergency_stop");

    programmedPositionClient_ = create_client<msg_srv::srv::MoveToPosition>("programmed_position");
}

CommunicatorNode::~CommunicatorNode()
{
}

void CommunicatorNode::sendSingleServoCommand(short servo, unsigned long long angle, unsigned long long movement, std::string movementType)
{
    auto request_ = std::make_shared<msg_srv::srv::SingleServoCommand::Request>();

    request_->position.target_servo = servo;
    request_->position.degrees = angle;
    request_->position.movement = movement;
    request_->position.movement_type = movementType;

    std::cout << "sending request: " << (short)request_->position.target_servo << " - " << request_->position.degrees << " - " << request_->position.movement << " - " << request_->position.movement_type << std::endl;
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

void CommunicatorNode::sendMultiServoCommand(std::vector<std::vector<long long>> arguments, std::vector<std::string> movementTypes)
{
    auto request_ = std::make_shared<msg_srv::srv::MultiServoCommand::Request>();
    msg_srv::msg::Move move;

    for (long unsigned int i = 0; i < arguments.size(); i++)
    {
        msg_srv::msg::ServoCommand servoCommand;
        servoCommand.target_servo = arguments.at(i).at(0);
        servoCommand.degrees = arguments.at(i).at(1);
        servoCommand.movement = arguments.at(i).at(2);
        servoCommand.movement_type = movementTypes.at(i);
        move.instruction.push_back(servoCommand);
    }

    request_->positions = move;
    auto result = multiServoClient_->async_send_request(request_);

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

void CommunicatorNode::sendStopCommand()
{
    auto request_ = std::make_shared<msg_srv::srv::EmergencyStop::Request>();
    auto result = stopClient_->async_send_request(request_);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(get_logger(), "Sheesh %x Wazza", result.get()->stopped);
    }
    else
    {
        RCLCPP_ERROR(get_logger(),
                     "Failed to call service ");
    }
}

void CommunicatorNode::sendProgrammedPositionCommand(std::string programmedPosition)
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
