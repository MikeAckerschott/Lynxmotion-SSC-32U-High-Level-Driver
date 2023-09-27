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

  context = new Context(new idleState, serial_, std::shared_ptr<HighLevelNode>(this));

  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]()
                             { context->f_do(); });
}

HighLevelNode::~HighLevelNode()
{
}

void HighLevelNode::handleEmergencyStop(const std::shared_ptr<msg_srv::srv::EmergencyStop::Request> request,
                                        const std::shared_ptr<msg_srv::srv::EmergencyStop::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Emergency stop received");

  LowLevelServer::handleEmergencyStop(serial_);

  response->stopped = true;
}

void HighLevelNode::handleSingleServoServiceRequest(const std::shared_ptr<msg_srv::srv::SingleServoCommand::Request> request,
                                                    const std::shared_ptr<msg_srv::srv::SingleServoCommand::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Single servo command received");

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
    RCLCPP_INFO(this->get_logger(), "Movement type not recognized");
    response->finished = false;
    return;
  }

  if (!ServoUtils::verifyServoConstraints(targetServo, position))
  {
    response->finished = false;
    RCLCPP_INFO(this->get_logger(), "Servo or position out of bounds");
    return;
  }

  position = ServoUtils::degreesToPwm(targetServo, position);

  SingleServoCommand command(targetServo, position, movement, type, serial_);
  command.sendCommand();
  response->finished = true;
}

void HighLevelNode::handleMultiServoServiceRequest(const std::shared_ptr<msg_srv::srv::MultiServoCommand::Request> request,
                                                   const std::shared_ptr<msg_srv::srv::MultiServoCommand::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Multi servo command received");

  msg_srv::msg::Move move = request->positions;

  RCLCPP_INFO(this->get_logger(), "Number of instructions: %ld", move.instruction.size());

  std::vector<SingleServoCommand> commands;

  for (long unsigned int i = 0; i < move.instruction.size(); ++i)
  {
    short targetServo = (short)move.instruction.at(i).target_servo;
    std::cout << targetServo << std::endl;
    long long position = move.instruction.at(i).degrees;
    std::cout << position << std::endl;
    long long movement = move.instruction.at(i).movement;
    std::cout << movement << std::endl;
    std::string movementType = move.instruction.at(i).movement_type;
    std::cout << movementType << std::endl;

    SingleServoCommand::movementType type = ServoUtils::getMovementType(movementType);

    if (movement == 0)
    {
      response->finished = false;
      RCLCPP_INFO(this->get_logger(), "Movement cannot be 0");
      return;
    }

    std::cout << "movement type verified" << std::endl;

    if (!ServoUtils::verifyServoConstraints(targetServo, position))
    {
      response->finished = false;
      RCLCPP_INFO(this->get_logger(), "Servo or position out of bounds");
      return;
    }

    std::cout << "servo constraints verified" << std::endl;

    short positionAsPWM = ServoUtils::degreesToPwm(targetServo, position);

    std::cout << "position as pwm calculated" << std::endl;
    commands.push_back(SingleServoCommand(targetServo, positionAsPWM, movement, type, serial_));
  }
  MultiServoCommand command(commands, serial_);
  command.sendCommand();
  response->finished = true;
}

void HighLevelNode::handleProgrammedPosition(const std::shared_ptr<msg_srv::srv::MoveToPosition::Request> request,
                                             const std::shared_ptr<msg_srv::srv::MoveToPosition::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Programmed position received");

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
  else
  {
    RCLCPP_INFO(this->get_logger(), "ProgrammedPosition not recognized");
    response->finished = false;
    return;
  }

  context->programmedPositionCommandReceived = true;

  Command command(commandAsString, serial_);
  command.sendCommand();
  response->finished = true;
}
