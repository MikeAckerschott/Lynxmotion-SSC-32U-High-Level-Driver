#include "highLevelNode.hpp"
#include "idleState.hpp"
#include "movingState.hpp"

void idleState::f_entry()
{
    RCLCPP_INFO(context_->logger_, "STATE: {idleState}");
}

idleState::idleState()
{
}

void idleState::f_do()
{
}

void idleState::f_exit()
{
}

bool idleState::singleServoCommandReceived()
{
    return context_->singleServoCommandReceived;
}

bool idleState::multiServoCommandReceived()
{
    return context_->multiServoCommandReceived;
}

bool idleState::emergencyStopReceived()
{
    return context_->emergencyStopActivateRequest;
}

bool idleState::programmedPositionCommandReceived()
{
    return context_->programmedPositionCommandReceived;
}

bool idleState::movementQueueNotEmpty()
{
    return !context_->commandQueue_.empty();
}

bool idleState::checkAllTriggers()
{
    if (emergencyStopReceived())
    {   
        context_->TransitionTo(new emergencyStopState);
        return true;
    }

    if(movementQueueNotEmpty()){
        context_->TransitionTo(new movingState);
        return true;
    }

    // if (singleServoCommandReceived())
    // {

    //     context_->singleServoCommandReceived = false;
    //     context_->TransitionTo(new movingState);
    //     return true;
    // }

    // if (multiServoCommandReceived())
    // {

    //     context_->multiServoCommandReceived = false;
    //     context_->TransitionTo(new movingState);
    //     return true;
    // }

    // if (programmedPositionCommandReceived())
    // {
    //     context_->programmedPositionCommandReceived = false;
    //     context_->TransitionTo(new movingState);
    //     return true;
    // }

    return false;
}