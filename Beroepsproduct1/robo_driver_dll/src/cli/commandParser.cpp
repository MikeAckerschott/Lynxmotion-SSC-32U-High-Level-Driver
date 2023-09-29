#include "commandParser.hpp"
#include <sstream>
#include <iostream>
#include <vector>

CommandParser::CommandParser(std::shared_ptr<CommunicatorNode> node) : communicatorNode_(node) {}

CommandParser::~CommandParser() {}

void CommandParser::parseCommand(std::string command)
{
    // get first word from command
    if (command.size() < 4)
    {
        RCLCPP_ERROR(communicatorNode_->get_logger(), "Command too short. expects at least 4 characters but got %ld", command.size());
        return;
    }

    std::string commandType = command.substr(0, command.find(" "));
    if (!parseSingleServoCommand(commandType, command) && !parseMultiServoCommand(commandType, command) && !parseStopCommand(commandType) && !parseProgrammedPositionCommand(commandType, command) && !parseStartCommand(commandType) && !parseSkipCommand(commandType) && !parseEmptyQueueCommand(commandType))
    {
        RCLCPP_ERROR(communicatorNode_->get_logger(), "Command not recognized. supported commands are: singleServo, multiServo, stop, programmedPosition. check README for syntax");
        return;
    }
}

bool CommandParser::parseSingleServoCommand(std::string commandType, std::string command)
{
    if (commandType != "singleServo")
    {
        return false;
    }

    std::string commandArguments = command.substr(command.find(" ") + 1, command.size());

    int servoNumber;
    int angle;
    int movement;
    std::string movementType;

    if (!getSingleServoCommandArguments(commandArguments, servoNumber, angle, movement, movementType))
    {

        RCLCPP_ERROR(communicatorNode_->get_logger(), "Could not parse command arguments. check README for syntax");
        return false;
    }

    // Dont check, hld will do this

    RCLCPP_INFO(communicatorNode_->get_logger(), "Sending singleServoCommand... servoNumber: %d angle: %d movement: %d movementType: %s", servoNumber, angle, movement, movementType.c_str());
    communicatorNode_->sendSingleServoCommand(servoNumber, angle, movement, movementType);
    return true;
}

bool CommandParser::parseMultiServoCommand(std::string commandType, std::string command)
{
    if (commandType != "multiServo")
    {
        return false;
    }

    std::string commandArguments = command.substr(command.find(" ") + 1, command.size());

    // parse mutli servo commands
    // multi servo commands look like this: "multiServo {servo:0 angle:0 speed:10} {servo:1 angle:0 speed:10} ...etc"

    long unsigned int firstBracket = commandArguments.find("{");
    long unsigned int lastBracket = commandArguments.find("}");

    std::vector<std::vector<long long>> servoParams;
    std::vector<std::string> movementTypes;

    while (firstBracket != std::string::npos && lastBracket != std::string::npos)
    {
        std::string singleServoCommand = commandArguments.substr(firstBracket + 1, lastBracket - firstBracket - 1);

        int servoNumber;
        int angle;
        int movement;
        std::string movementType;

        if (!getSingleServoCommandArguments(singleServoCommand, servoNumber, angle, movement, movementType))
        {
            RCLCPP_ERROR(communicatorNode_->get_logger(), "Could not parse command arguments. check README for syntax");
            return false;
        }

        servoParams.push_back(std::vector<long long>{servoNumber, angle, movement});
        movementTypes.push_back(movementType);

        // Dont check, hld will do this
        firstBracket = commandArguments.find("{", lastBracket);
        lastBracket = commandArguments.find("}", firstBracket);
    }

    if (servoParams.size() == 0)
    {
        RCLCPP_ERROR(communicatorNode_->get_logger(), "No servo params found");
        return false;
    }

    communicatorNode_->sendMultiServoCommand(servoParams, movementTypes);
    return true;
}

bool CommandParser::parseStopCommand(std::string commandType)
{
    if (commandType != "stop")
    {
        return false;
    }

    communicatorNode_->sendStopCommand();
    return true;
}

bool CommandParser::parseStartCommand(std::string commandType)
{
    if (commandType != "start")
    {
        return false;
    }

    communicatorNode_->deactivateEmergencyStop();
    return true;
}

bool CommandParser::parseProgrammedPositionCommand(std::string commandType, std::string command)
{
    if (commandType != "programmedPosition")
    {
        return false;
    }

    std::string commandArguments = command.substr(command.find(" ") + 1, command.size());

    if (commandArguments != "park" && commandArguments != "ready" && commandArguments != "straight-up")
    {
        RCLCPP_ERROR(communicatorNode_->get_logger(), "ProgrammedPosition not recognized, supported positions are: park, ready, straight-up");
        return false;
    }

    communicatorNode_->sendProgrammedPositionCommand(commandArguments);
    return true;
}

bool CommandParser::parseSkipCommand(std::string commandType)
{
    if (commandType != "skip")
    {
        return false;
    }

    communicatorNode_->sendSkipCommand();
    return true;
}

bool CommandParser::isNumber(const std::string &s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && (std::isdigit(*it) || *it == '-'))
        ++it;
    return !s.empty() && it == s.end();
}

bool CommandParser::getSingleServoCommandArguments(std::string commandArguments, int &servoNumber, int &angle, int &movement, std::string &movementType)
{
    // Command looks like this: "singleServo servo:0 angle:0 speed:10 duration:1000"
    // the arguments dont have to be in this order, but they do have to be seperated by a space
    // We need to extract the servo number, angle, speed and duration from this string

    bool servoNumberFound = commandArguments.find("servo:") != std::string::npos;
    bool angleFound = commandArguments.find("angle:") != std::string::npos;
    bool speedFound = commandArguments.find("speed:") != std::string::npos;
    bool durationFound = commandArguments.find("duration:") != std::string::npos;

    if (!servoNumberFound || !angleFound)
    {
        RCLCPP_ERROR(communicatorNode_->get_logger(), "Servo or angle param missing");
        return false;
    }

    if (speedFound && durationFound)
    {
        RCLCPP_ERROR(communicatorNode_->get_logger(), "Speed and duration cant both be set");
        return false;
    }

    std::string servoNumberString = commandArguments.substr(commandArguments.find("servo:") + 6, commandArguments.find(" ", commandArguments.find("servo:")) - commandArguments.find("servo:") - 6);
    std::string angleString = commandArguments.substr(commandArguments.find("angle:") + 6, commandArguments.find(" ", commandArguments.find("angle:")) - commandArguments.find("angle:") - 6);
    std::string speedString = "";
    std::string durationString = "";
    if (speedFound)
    {
        speedString = commandArguments.substr(commandArguments.find("speed:") + 6, commandArguments.find(" ", commandArguments.find("speed:")) - commandArguments.find("speed:") - 6);
    }
    if (durationFound)
    {
        durationString = commandArguments.substr(commandArguments.find("duration:") + 9, commandArguments.find(" ", commandArguments.find("duration:")) - commandArguments.find("duration:") - 9);
    }
    // check if string is numeric

    if (!isNumber(servoNumberString))
    {
        RCLCPP_ERROR(communicatorNode_->get_logger(), "Servo number is not numeric");
        return false;
    }

    if (!isNumber(angleString))
    {
        RCLCPP_ERROR(communicatorNode_->get_logger(), "Angle is not numeric");
        return false;
    }

    if (speedFound && !isNumber(speedString))
    {
        RCLCPP_ERROR(communicatorNode_->get_logger(), "Speed is not numeric");
        return false;
    }
    else if (speedFound)
    {
        movement = std::stoi(speedString);
        movementType = "speed";
    }

    if (durationFound && !isNumber(durationString))
    {
        RCLCPP_ERROR(communicatorNode_->get_logger(), "Duration is not numeric");
        return false;
    }
    else if (durationFound)
    {
        movement = std::stoi(durationString);
        movementType = "duration";
    }

    std::cout << "servo: " << servoNumberString << " angle: " << angleString << " speed: " << speedString << " duration: " << durationString << std::endl;

    servoNumber = std::stoi(servoNumberString);
    angle = std::stoi(angleString);

    return true;
}

bool CommandParser::parseEmptyQueueCommand(std::string commandType)
{
    if (commandType != "emptyQueue")
    {
        return false;
    }

    communicatorNode_->sendEmptyQueueCommand();
    return true;
}
