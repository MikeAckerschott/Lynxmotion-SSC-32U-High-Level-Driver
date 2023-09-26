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
    if (!parseSingleServoCommand(commandType, command, node) && !parseMultiServoCommand(commandType, command, node) && !parseStopCommand(commandType, node) && !parseProgrammedPositionCommand(commandType, command, node))
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

    int servoNumber;
    int angle;
    int movement;
    std::string movementType;

    if (!getSingleServoCommandArguments(commandArguments, servoNumber, angle, movement, movementType))
    {
        std::cout << "Could not parse command arguments" << std::endl;
        return false;
    }

    // Dont check, hld will do this

    std::cout << "servoNumber: " << servoNumber << " angle: " << angle << " movement: " << movement << " movementType: " << movementType << std::endl;

    node->sendSingleServoCommand(servoNumber, angle, movement, movementType);
    return true;
}

bool CommandParser::parseMultiServoCommand(std::string commandType, std::string command, std::shared_ptr<CommunicatorNode> node)
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
            std::cout << "Could not parse command arguments" << std::endl;
            return false;
        }

        servoParams.push_back(std::vector<long long>{servoNumber, angle, movement});
        movementTypes.push_back(movementType);

        // Dont check, hld will do this

        std::cout << "servoNumber: " << servoNumber << " angle: " << angle << " movement: " << movement << " movementType: " << movementType << std::endl;

        firstBracket = commandArguments.find("{", lastBracket);
        lastBracket = commandArguments.find("}", firstBracket);
    }

    if (servoParams.size() == 0)
    {
        std::cout << "No servo params found" << std::endl;
        return false;
    }

    node->sendMultiServoCommand(servoParams, movementTypes);
return true;
}

bool CommandParser::parseStopCommand(std::string commandType, std::shared_ptr<CommunicatorNode> node)
{
    if (commandType != "stop")
    {
        return false;
    }

    node->sendStopCommand();
    return true;
}

bool CommandParser::parseProgrammedPositionCommand(std::string commandType, std::string command, std::shared_ptr<CommunicatorNode> node)
{
    if (commandType != "programmedPosition")
    {
        return false;
    }

    std::string commandArguments = command.substr(command.find(" ") + 1, command.size());

    std::cout << "'" << commandArguments << "'" << std::endl;

    if (commandArguments != "park" && commandArguments != "ready" && commandArguments != "straight-up")
    {
        std::cout << "ProgrammedPosition not recognized" << std::endl;
        return false;
    }

    node->sendProgrammedPositionCommand(commandArguments);
return true;
}

bool CommandParser::isNumber(const std::string &s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it))
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
        std::cout << "Servo or angle param missing" << std::endl;
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
    else if (speedFound)
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
    else if (durationFound)
    {
        std::cout << "test2" << std::endl;
        movement = std::stoi(durationString);
        movementType = "duration";
    }

    std::cout << "test" << std::endl;

    servoNumber = std::stoi(servoNumberString);
    angle = std::stoi(angleString);

    return true;
}
