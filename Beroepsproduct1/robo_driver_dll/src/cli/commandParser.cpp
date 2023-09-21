#include "commandParser.hpp"
#include <sstream>
#include <iostream>
#include <vector>

CommandParser::CommandParser() {}

CommandParser::~CommandParser() {}

void CommandParser::parseCommand(std::string command, std::shared_ptr<CommunicatorNode> node)
{
    // get first word from command
    if (command.size() < 4)
    {
        std::cout << "Command too short" << std::endl;
        return;
    }

    std::string commandType = command.substr(0, command.find(" "));
    if (!parseSingleServoCommand(commandType, command, node) && !parseMultiServoCommand(commandType, command, node) && !parseStopCommand(commandType, command, node) && !parseProgrammedPositionCommand(commandType, command, node))
    {
        std::cout << "Command not recognized" << std::endl;
        return;
    }
}

bool CommandParser::parseSingleServoCommand(std::string commandType, std::string command, std::shared_ptr<CommunicatorNode> node)
{
    if (commandType != "singleServo")
    {
        return false;
    }

    std::string commandArguments = command.substr(command.find(" ") + 1, command.size());

    // Command looks like this: "singleServo servo:0 angle:0 speed:10 duration:1000"
    // the arguments dont have to be in this order, but they do have to be seperated by a space
    // We need to extract the servo number, angle, speed and duration from this string

    bool servoNumberFound = commandArguments.find("servo:") != std::string::npos;
    bool angleFound = commandArguments.find("angle:") != std::string::npos;
    bool speedFound = commandArguments.find("speed:") != std::string::npos;
    bool durationFound = commandArguments.find("duration:") != std::string::npos;

    if (!servoNumberFound || !angleFound)
    {
        std::cout << "Servo or angle param missing" << std::endl;
        return false;
    }

    if (speedFound == false && durationFound == false)
    {
        std::cout << "speed and duration cant not both be set" << std::endl;
        return false;
    }

    if (speedFound && durationFound)
    {
        std::cout << "speed and duration cant both be set" << std::endl;
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

    std::cout << "servo: '" << servoNumberString << "' angle: '" << angleString << "' speed: '" << speedString << "' duration: '" << durationString << "'" << std::endl;

    // convert the strings to integers, but first check if they are valid integers

    int servoNumber;
    int angle;
    int movement;
    std::string movementType;

    // check if string is numeric

    if (!isNumber(servoNumberString))
    {
        std::cout << "servoNumber is not numeric" << std::endl;
        return false;
    }

    if (!isNumber(angleString))
    {
        std::cout << "angle is not numeric" << std::endl;
        return false;
    }

    if (speedFound && !isNumber(speedString))
    {
        std::cout << "speed is not numeric" << std::endl;
        return false;
    }
    else if(speedFound)
    {
        std::cout << "test3" << std::endl;
        movement = std::stoi(speedString);
        movementType = "speed";
    }

    if (durationFound && !isNumber(durationString))
    {
        std::cout << "duration is not numeric" << std::endl;
        return false;
    }
    else if(durationFound)
    {
        std::cout << "test2" << std::endl;
        movement = std::stoi(durationString);
        movementType = "duration";
    }

    std::cout << "test" << std::endl;

    servoNumber = std::stoi(servoNumberString);
    angle = std::stoi(angleString);

    // Dont check, hld will do this

    node->sendSingleServoCommand(servoNumber, angle, movement, movementType);
}

bool CommandParser::parseMultiServoCommand(std::string commandType, std::string command, std::shared_ptr<CommunicatorNode> node)
{
    return true;
}

bool CommandParser::parseStopCommand(std::string commandType, std::string command, std::shared_ptr<CommunicatorNode> node)
{
    return true;
}

bool CommandParser::parseProgrammedPositionCommand(std::string commandType, std::string command, std::shared_ptr<CommunicatorNode> node)
{
    return true;
}

bool CommandParser::isNumber(const std::string &s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it))
        ++it;
    return !s.empty() && it == s.end();
}
