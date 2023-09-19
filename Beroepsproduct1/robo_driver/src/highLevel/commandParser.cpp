#include "commandParser.hpp"
#include <sstream>
#include <iostream>
#include <vector>

CommandParser::CommandParser() {}

CommandParser::~CommandParser() {}

void CommandParser::parseCommand(std::string command, std::shared_ptr<HighLevelNode> node)
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

bool CommandParser::parseSingleServoCommand(std::string commandType, std::string command, std::shared_ptr<HighLevelNode> node)
{
    if (commandType != "singleServo")
    {
        return false;
    }

    std::string commandArguments = command.substr(command.find(" ") + 1, command.size());

    // auto servoCommand = std::make_shared<low_level_driver::srv::SingleServoCommand::Request>();
    std::vector<long long> arguments;
    std::istringstream iss(commandArguments);

    std::string token;
    while (std::getline(iss, token, ' '))
    {
        // Convert the token to an integer and add it to the vector
        long long number;
        std::istringstream(token) >> number;
        arguments.push_back(number);
    }

    if (arguments.size() != 4)
    {
        std::cout << "Wrong number of arguments. Got: " << arguments.size() << ". expected 4. " << std::endl;
        return false;
    }
    node->sendSingleServoCommand(arguments[0], arguments[1], arguments[2], arguments[3]);
    return true;
}

bool CommandParser::parseMultiServoCommand(std::string commandType, std::string command, std::shared_ptr<HighLevelNode> node)
{
    if (commandType != "multiServo")
    {
        return false;
    }

    std::cout << "parsing multiServo" << std::endl;

    std::string commandArguments = command.substr(command.find(" ") + 1, command.size());
    int start = 0, end = 0;
    std::vector<std::string> servoCommandStrings;
    while ((end = commandArguments.find("}", start)) != std::string::npos)
    {
        std::string servoCommandString = commandArguments.substr(start + 1, end - start - 1);
        servoCommandStrings.push_back(servoCommandString);
        start = commandArguments.find("{", end);
    }

    std::cout << "servoCommandStrings size: " << servoCommandStrings.size() << std::endl;

    // get 4 integers from command. if it is more or less return false
    std::vector<std::vector<long long>> multiServoArguments;
    short iterator = 0;

    for (std::string argument : servoCommandStrings)
    {
        std::vector<long long> singleServoArguments;

        std::istringstream iss(argument);
        std::string token;
        while (std::getline(iss, token, ' '))
        {
            // Convert the token to an integer and add it to the vector

            long long number;
            std::istringstream(token) >> number;
            // std::cout<<number<<std::endl;
            singleServoArguments.push_back(number);
        }

        if (singleServoArguments.size() != 4)
        {
            std::cout << "Wrong number of arguments. Got: " << singleServoArguments.size() << ". expected 4. " << std::endl;
            return false;
        }
        multiServoArguments.push_back(singleServoArguments);
    }

    std::cout << "sending request" << std::endl;

    node->sendMultiServoCommand(multiServoArguments);
    return true;
}

bool CommandParser::parseStopCommand(std::string commandType, std::string command, std::shared_ptr<HighLevelNode> node)
{
    if (command != "stop")
    {
        return false;
    }
    node->sendStopCommand();
    return true;
}

bool CommandParser::parseProgrammedPositionCommand(std::string commandType, std::string command, std::shared_ptr<HighLevelNode> node)
{
    if (commandType != "programmedPosition")
    {
        return false;
    }

    std::string position = command.substr(command.find(" ") + 1, command.size());
    std::cout << "position: " << position <<"."<< std::endl;

    node->sendProgrammedPositionCommand(position);
    return true;
}
