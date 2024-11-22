#include "highLevelNode.hpp"
#include <cstdint>

using namespace std::chrono_literals;

HighLevelNode::HighLevelNode() : Node("high_level_client"), serial_(ioservice, "/dev/ttyUSB0")
{
  singleServoService = create_service<msg_srv::srv::SingleServoCommand>("single_servo_command", std::bind(&HighLevelNode::handleSingleServoServiceRequest, this,
                                                                                                          std::placeholders::_1, std::placeholders::_2));

  multiServoService = create_service<msg_srv::srv::MultiServoCommand>(
      "multi_servo_command",
      std::bind(&HighLevelNode::handleMultiServoServiceRequest, this,
                std::placeholders::_1, std::placeholders::_2));

  stopService = create_service<msg_srv::srv::EmergencyStop>("emergency_stop", std::bind(&HighLevelNode::handleEmergencyStop, this,
                                                                                        std::placeholders::_1, std::placeholders::_2));

  programmedPositionService = create_service<msg_srv::srv::MoveToPosition>("programmed_position", std::bind(&HighLevelNode::handleProgrammedPosition, this,
                                                                                                            std::placeholders::_1, std::placeholders::_2));

  skipService = create_service<msg_srv::srv::Skip>("skip", std::bind(&HighLevelNode::handleSkip, this,
                                                                     std::placeholders::_1, std::placeholders::_2));

  emptyQueueService = create_service<msg_srv::srv::EmptyQueue>("empty_queue", std::bind(&HighLevelNode::handleEmptyQueue, this,
                                                                                        std::placeholders::_1, std::placeholders::_2));
  auto ret = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
  
  context = new Context(new idleState, serial_, get_logger());

  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]()
                             { context->f_do(); });
}

HighLevelNode::~HighLevelNode()
{
}

void HighLevelNode::handleEmergencyStop(const std::shared_ptr<msg_srv::srv::EmergencyStop::Request> request,
                                        const std::shared_ptr<msg_srv::srv::EmergencyStop::Response> response)
{

  if (request->enable == true)
  {
    RCLCPP_DEBUG(get_logger(), "EVENT: {Emergency stop | enable: true}");
    context->emergencyStopActivateRequest = true;
    response->stopped = true;
    LowLevelServer::stopCurrentMovement(serial_);
  }
  else
  {
    RCLCPP_DEBUG(get_logger(), "EVENT: {Emergency stop | enable: false}");
    context->emergencyStopDeactivateRequest = true;
    response->stopped = false;
  }
}

void HighLevelNode::handleSkip(const std::shared_ptr<msg_srv::srv::Skip::Request> request,
                               const std::shared_ptr<msg_srv::srv::Skip::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "EVENT: {Skip}");
  LowLevelServer::stopCurrentMovement(serial_);

  context->skipCommandReceived = true;
  if (context->commandQueue_.empty())
  {
    response->empty = true;
    return;
  }
  response->empty = false;
}

void HighLevelNode::handleSingleServoServiceRequest(const std::shared_ptr<msg_srv::srv::SingleServoCommand::Request> request,
                                                    const std::shared_ptr<msg_srv::srv::SingleServoCommand::Response> response)
{

  short targetServo = (short)request->position.target_servo;
  long long position = request->position.degrees;
  long long movement = request->position.movement;
  std::string movementType = request->position.movement_type;

  SingleServoCommand::movementType type;
  if (movementType == "duration")
  {
    type = SingleServoCommand::MOVE_WITH_TIME;
  }
  else if (movementType == "speed")
  {
    type = SingleServoCommand::MOVE_WITH_SPEED;
  }
  else if (movementType == "")
  {
    type = SingleServoCommand::NO_TYPE;
  }
  else
  {
    response->finished = false;
    return;
  }

  if (!ServoUtils::verifyServoConstraints(targetServo, position))
  {
    response->finished = false;
    return;
  }

  position = ServoUtils::degreesToPwm(targetServo, position);

  RCLCPP_DEBUG(get_logger(), "EVENT: {Single servo command | servo: %d, angle: %llu, movement: %llu, movementType: %s}", targetServo, position, movement, movementType.c_str());
  context->singleServoCommandReceived = true;

  Command command = SingleServoCommand(targetServo, position, movement, type, serial_);
  context->commandQueue_.push(command);

  response->finished = true;
}

void HighLevelNode::handleMultiServoServiceRequest(const std::shared_ptr<msg_srv::srv::MultiServoCommand::Request> request,
                                                   const std::shared_ptr<msg_srv::srv::MultiServoCommand::Response> response)
{

  msg_srv::msg::Move move = request->positions;

  std::vector<SingleServoCommand> commands;

  for (long unsigned int i = 0; i < move.instruction.size(); ++i)
  {
    short targetServo = (short)move.instruction.at(i).target_servo;
    long long position = move.instruction.at(i).degrees;
    long long movement = move.instruction.at(i).movement;
    std::string movementType = move.instruction.at(i).movement_type;

    SingleServoCommand::movementType type = ServoUtils::getMovementType(movementType);

    if (movement == 0)
    {
      response->finished = false;
      return;
    }

    if (!ServoUtils::verifyServoConstraints(targetServo, position))
    {
      response->finished = false;
      return;
    }

    short positionAsPWM = ServoUtils::degreesToPwm(targetServo, position);
    commands.push_back(SingleServoCommand(targetServo, positionAsPWM, movement, type, serial_));
  }

  RCLCPP_DEBUG(get_logger(), "EVENT: {Multi servo command}");
  context->multiServoCommandReceived = true;

  Command command = MultiServoCommand(commands, serial_);
  context->commandQueue_.push(command);

  response->finished = true;
}

void HighLevelNode::handleProgrammedPosition(const std::shared_ptr<msg_srv::srv::MoveToPosition::Request> request,
                                             const std::shared_ptr<msg_srv::srv::MoveToPosition::Response> response)
{

  std::string commandAsString;

  if (request->position.programmed_position == "park")
  {
    commandAsString = CommandUtils::park;
    commandAsString += Command::cr;
  }
  else if (request->position.programmed_position == "ready")
  {
    commandAsString = CommandUtils::ready;
    commandAsString += Command::cr;
  }
  else if (request->position.programmed_position == "straight-up")
  {
    commandAsString = CommandUtils::straightUp;
    commandAsString += Command::cr;
  }
  else if (request->position.programmed_position == "wristLeft")
  {
    commandAsString = CommandUtils::wristLeft;
    commandAsString += Command::cr;
  }
  else if (request->position.programmed_position == "wristRight")
  {
    commandAsString = CommandUtils::wristRight;
    commandAsString += Command::cr;
  }
  else if (request->position.programmed_position == "wristMiddle")
  {
    commandAsString = CommandUtils::wristMiddle;
    commandAsString += Command::cr;
  }
  else if (request->position.programmed_position == "parkLeft")
  {
    commandAsString = CommandUtils::parkLeft;
    commandAsString += Command::cr;
  }
  else if (request->position.programmed_position == "parkRight")
  {
    commandAsString = CommandUtils::parkRight;
    commandAsString += Command::cr;
  }
  else if (request->position.programmed_position == "elbowShakingUp")
  {
    commandAsString = CommandUtils::elbowShakingUp;
    commandAsString += Command::cr;
  }
  else if (request->position.programmed_position == "elbowShakingDown")
  {
    commandAsString = CommandUtils::elbowShakingDown;
    commandAsString += Command::cr;
  }
  else
  {
    response->finished = false;
    return;
  }

  context->programmedPositionCommandReceived = true;
  RCLCPP_DEBUG(get_logger(), "EVENT: {Programmed position command | position: %s}", request->position.programmed_position.c_str());

  Command command(commandAsString, serial_);
  context->commandQueue_.push(command);
  response->finished = true;
}

void HighLevelNode::handleEmptyQueue(const std::shared_ptr<msg_srv::srv::EmptyQueue::Request> request,
                                     const std::shared_ptr<msg_srv::srv::EmptyQueue::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "EVENT: {Empty queue}");
  context->emptyQueueCommandReceived = true;

  if (context->commandQueue_.empty())
  {
    response->empty = true;
    return;
  }

  response->empty = false;
}
