#include "lowLevelServer.hpp"
#include "servoUtils.hpp"
#include "singleServoCommand.hpp"
#include "multiServoCommand.hpp"
#include "commandUtils.hpp"
#include <boost/asio.hpp>

LowLevelServer::LowLevelServer()
    : Node("low_level_server"), serial_(ioservice, "/dev/ttyUSB0")
{
  // log hello world
  RCLCPP_INFO(this->get_logger(), "Hello world from LowLevelServer");

  // msg_srv::srv::SingleServoCommand;

  singleServoService = create_service<msg_srv::srv::SingleServoCommand>("single_servo_command", std::bind(&LowLevelServer::handleSingleServoServiceRequest, this,
                                                                                                                   std::placeholders::_1, std::placeholders::_2));

  multiServoService = create_service<msg_srv::srv::MultiServoCommand>(
      "multi_servo_command",
      std::bind(&LowLevelServer::handleMultiServoServiceRequest, this,
                std::placeholders::_1, std::placeholders::_2));

  stopService = create_service<msg_srv::srv::EmergencyStop>("emergency_stop", std::bind(&LowLevelServer::handleEmergencyStop, this,
                                                                                                 std::placeholders::_1, std::placeholders::_2));
  programmedPositionService = create_service<msg_srv::srv::MoveToPosition>("programmed_position", std::bind(&LowLevelServer::handleProgrammedPosition, this,
                                                                                                                     std::placeholders::_1, std::placeholders::_2));

  serial_.set_option(boost::asio::serial_port_base::baud_rate(9600));
  serial_.set_option(boost::asio::serial_port::flow_control(
      boost::asio::serial_port::flow_control::none));
  serial_.set_option(
      boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  serial_.set_option(boost::asio::serial_port::stop_bits(
      boost::asio::serial_port::stop_bits::one));
  serial_.set_option(boost::asio::serial_port::character_size(
      boost::asio::serial_port::character_size(8)));
}

LowLevelServer::~LowLevelServer()
{
}

void LowLevelServer::handleSingleServoServiceRequest(
    const std::shared_ptr<msg_srv::srv::SingleServoCommand::Request>
        request,
    const std::shared_ptr<msg_srv::srv::SingleServoCommand::Response>
        response)
{

  short targetServo = (short)request->position.target_servo;
  long long position = request->position.position;
  long long duration = request->position.duration;
  long long speed = request->position.speed;

  std::cout << "request info: " << (short)targetServo << " - " << position
            << " - " << duration << " - " << speed
            << std::endl;

  if (!ServoUtils::verifyServoConstraints(targetServo, position))
  {
    response->finished = false;
    std::cout << "servo or position out of bounds" << std::endl;
    return;
  }

  if (!ServoUtils::verifyMovementType(duration, speed))
  {
    response->finished = false;
    std::cout << "speed out of bounds" << std::endl;
    return;
  }

  short positionAsPWM = static_cast<short>(ServoUtils::degreesToPwm(targetServo, position));

  std::cout << "positionAsPWM: " << positionAsPWM << std::endl;

  SingleServoCommand::movementType movementType;
  long long movement = 0;
  if (duration > 0)
  {
    movement = duration;
    movementType = SingleServoCommand::MOVE_WITH_TIME;
  }
  else
  {
    movement = speed;
    movementType = SingleServoCommand::MOVE_WITH_SPEED;
  }

  Command command = SingleServoCommand(targetServo, positionAsPWM, movement,
                                       movementType, serial_);

  command.sendCommand();

  response->finished = true;
  // This function is called when a client sends a request to the service
  // You can process the request here and send a response back to the client
}

void LowLevelServer::handleMultiServoServiceRequest(
    const std::shared_ptr<msg_srv::srv::MultiServoCommand::Request>
        request,
    const std::shared_ptr<msg_srv::srv::MultiServoCommand::Response>
        response)
{
  msg_srv::msg::Move move = request->positions;

  std::vector<SingleServoCommand> commands;

  for (int i = 0; i < move.instruction.size(); ++i)
  {
    short targetServo = (short)move.instruction.at(i).target_servo;
    long long position = move.instruction.at(i).position;
    long long duration = move.instruction.at(i).duration;
    long long speed = move.instruction.at(i).speed;

    std::cout << "servo: " << targetServo << " - " << position << " - " << duration << " - " << speed << std::endl;

    if (!ServoUtils::verifyServoConstraints(targetServo, position))
    {
      response->finished = false;
      std::cout << "servo or position out of bounds" << std::endl;
      return;
    }

    if (!ServoUtils::verifyMovementType(duration, speed))
    {
      response->finished = false;
      std::cout << "speed out of bounds" << std::endl;
      return;
    }
    short positionAsPWM = static_cast<short>(ServoUtils::degreesToPwm(targetServo, position));
    SingleServoCommand::movementType movementType;
    long long movement = 0;
    if (duration > 0)
    {
      std::cout << "duration > 0" << std::endl;
      movementType = SingleServoCommand::MOVE_WITH_TIME;
      movement = duration;
    }
    else
    {
      std::cout << "duration < 0" << std::endl;
      movementType = SingleServoCommand::MOVE_WITH_SPEED;
      movement = speed;
    }

    commands.push_back(SingleServoCommand(targetServo, positionAsPWM, movement,
                                          movementType, serial_));
  }
  MultiServoCommand multiServoCommand(commands, serial_);
  multiServoCommand.sendCommand();
  response->finished = true;
}

void LowLevelServer::handleEmergencyStop(
    const std::shared_ptr<msg_srv::srv::EmergencyStop::Request> request,
    const std::shared_ptr<msg_srv::srv::EmergencyStop::Response> response)
{
  std::string commandAsString = "stop";
  commandAsString += Command::cr;

  std::cout << commandAsString << std::endl;
  Command command(commandAsString, serial_);
  command.sendCommand();

  response->stopped = true;
}

void LowLevelServer::handleProgrammedPosition(
    const std::shared_ptr<msg_srv::srv::MoveToPosition::Request> request,
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
  else
  {
    std::cout << "ProgrammedPosition not recognized" << std::endl;
    response->finished = false;
    return;
  }
  Command command(commandAsString, serial_);
  command.sendCommand();
  response->finished = true;
}