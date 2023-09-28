#include "movingState.hpp"

void movingState::f_entry()
{
    RCLCPP_INFO(context_->logger_, "STATE: {movingState}");
    context_->commandQueue_.front().sendCommand();
    context_->commandQueue_.pop();
}

movingState::movingState()
{
}

void movingState::f_do()
{
    if (isMovingDone())
    {
        if (!context_->commandQueue_.empty())
        {
            context_->commandQueue_.front().sendCommand();
            context_->commandQueue_.pop();
        }
    }
}

void movingState::f_exit()
{
}

bool movingState::isMovingDone()
{
    std::string commandString = "Q";
    commandString += Command::cr;

    Command command = Command(commandString, context_->serialPort_);
    command.sendCommand();

    char temp;
    temp = command.readMostRecentChar();

    return (temp == '.');
}

bool movingState::emergencyStopReceived()
{
    return context_->emergencyStopActivateRequest;
}

bool movingState::newPositionCommandReceived()
{
    return context_->programmedPositionCommandReceived;
}

bool movingState::newSingleServoCommandReceived()
{
    return context_->singleServoCommandReceived;
}

bool movingState::newMultiServoCommandReceived()
{
    return context_->multiServoCommandReceived;
}

bool movingState::isQueueEmpty()
{
    return context_->commandQueue_.empty();
}

bool movingState::checkAllTriggers()
{
    if (emergencyStopReceived())
    {
        context_->TransitionTo(new emergencyStopState);
        return true;
    }

    if (isQueueEmpty())
    {
        context_->TransitionTo(new idleState);
        return true;
    }
    // if (isMovingDone())
    // {
    //     context_->TransitionTo(new idleState);
    //     return true;
    // }

    // if(newPositionCommandReceived()){
    //     //TODO
    //     context_->programmedPositionCommandReceived = false;
    //     context_->TransitionTo(new movingState);
    //     return true;
    // }
    // if(newSingleServoCommandReceived()){
    //     //TODO
    //     context_->singleServoCommandReceived = false;
    //     context_->TransitionTo(new movingState);
    //     return true;
    // }
    // if(newMultiServoCommandReceived()){
    //     //TODO
    //     context_->multiServoCommandReceived = false;
    //     context_->TransitionTo(new movingState);
    //     return true;
    // }
    return false;
}