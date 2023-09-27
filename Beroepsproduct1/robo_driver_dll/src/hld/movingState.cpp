#include "movingState.hpp"

void movingState::f_entry()
{
    std::cout<<"MOVING ENTRY"<<std::endl;
}

movingState::movingState()
{
}

void movingState::f_do()
{
}

void movingState::f_exit()
{
}

bool movingState::isMovingDone()
{
    std::string commandString = "Q";
    commandString += Command::cr;
    Command command(commandString, context_->serialPort_);

    char temp;

    command.sendCommand();
    temp = command.readMostRecentChar();
    return (temp == '.');
}

bool movingState::emergencyStopReceived(){
    return context_->emergencyStopReceived;
}

bool movingState::checkAllTriggers()
{
    if (isMovingDone())
    {
        context_->TransitionTo(new idleState);
        return true;
    }
    if(emergencyStopReceived()){
        //TODO

    }
}