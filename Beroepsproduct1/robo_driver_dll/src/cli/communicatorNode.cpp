#include "communicatorNode.hpp"

CommunicatorNode::CommunicatorNode() : Node("communicator_node")
{
    singleServoClient_ = create_client<msg_srv::srv::SingleServoCommand>("single_servo_command");

    multiServoClient_ = create_client<msg_srv::srv::MultiServoCommand>("multi_servo_command");

    stopClient_ = create_client<msg_srv::srv::EmergencyStop>("emergency_stop");

    programmedPositionClient_ = create_client<msg_srv::srv::MoveToPosition>("programmed_position");

    skipClient_ = create_client<msg_srv::srv::Skip>("skip");

    emptyQueueClient_ = create_client<msg_srv::srv::EmptyQueue>("empty_queue");

    auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK)
    {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }
}

CommunicatorNode::~CommunicatorNode()
{
}

void CommunicatorNode::sendSingleServoCommand(int servo, int angle, int movement, std::string movementType)
{
    auto request_ = std::make_shared<msg_srv::srv::SingleServoCommand::Request>();

    request_->position.target_servo = servo;
    request_->position.degrees = angle;
    request_->position.movement = movement;
    request_->position.movement_type = movementType;

    RCLCPP_INFO(this->get_logger(), "Sending single servo command: servo: %d, angle: %u, movement: %u, movementType: %s", servo, angle, movement, movementType.c_str());

    auto result = singleServoClient_->async_send_request(request_);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->finished)
        {
            RCLCPP_INFO(get_logger(), "singleServoCommand received succesfully, moving robotic arm...");
        }
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
        if (result.get()->finished)
        {
            RCLCPP_INFO(get_logger(), "multiServoCommand received succesfully, moving robotic arm...");
        }
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

    request_->enable = true;

    auto result = stopClient_->async_send_request(request_);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->stopped)
        {
            RCLCPP_INFO(get_logger(), "emergencyStop received succesfully, stopping robotic arm...");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(),
                     "Failed to call service ");
    }
}

void CommunicatorNode::deactivateEmergencyStop()
{
    auto request_ = std::make_shared<msg_srv::srv::EmergencyStop::Request>();

    // set emergency stop enabled to false
    request_->enable = false;

    auto result = stopClient_->async_send_request(request_);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->stopped)
        {
            RCLCPP_INFO(get_logger(), "emergencyStop deactivation received succesfully, starting robotic arm...");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(),
                     "Failed to call service ");
    }
}

void CommunicatorNode::sendProgrammedPositionCommand(std::string programmedPosition)
{
    if (programmedPosition != "park" && programmedPosition != "ready" && programmedPosition != "straight-up")
    {
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
        if (result.get()->finished)
        {
            RCLCPP_INFO(get_logger(), "programmedPosition received succesfully, moving robotic arm...");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(),
                     "Failed to call service");
    }
}

void CommunicatorNode::sendSkipCommand()
{

    auto request_ = std::make_shared<msg_srv::srv::Skip::Request>();

    auto result = skipClient_->async_send_request(request_);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->empty)
        {
            RCLCPP_INFO(get_logger(), "skip received succesfully, but queue is empty");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "skip received succesfully, skipping command...");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(),
                     "Failed to call service ");
    }
}

void CommunicatorNode::sendEmptyQueueCommand()
{
    auto request_ = std::make_shared<msg_srv::srv::EmptyQueue::Request>();

    auto result = emptyQueueClient_->async_send_request(request_);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->empty)
        {
            RCLCPP_INFO(get_logger(), "emptyQueue received succesfully, but queue is empty");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "emptyQueue received succesfully, emptying queue...");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(),
                     "Failed to call service ");
    }
}