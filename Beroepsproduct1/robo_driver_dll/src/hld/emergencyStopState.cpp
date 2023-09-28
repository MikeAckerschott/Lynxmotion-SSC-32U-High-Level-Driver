#include "emergencyStopState.hpp"

void emergencyStopState::f_entry()
{
    RCLCPP_INFO(context_->logger_, "STATE: {emergencyStopState}");
    context_->emergencyStopActivateRequest = false;
    context_->emergencyStopDeactivateRequest = false;
}

emergencyStopState::emergencyStopState()
{
}

void emergencyStopState::f_do()
{
}

void emergencyStopState::f_exit()
{
    context_->emergencyStopActivateRequest = false;
    context_->emergencyStopDeactivateRequest = false;
    context_->commandQueue_ = std::queue<Command>();
}

bool emergencyStopState::deactivateEmergencyStopReceived()
{
    return context_->emergencyStopDeactivateRequest;
}

bool emergencyStopState::checkAllTriggers()
{
    if(deactivateEmergencyStopReceived()){
        context_->TransitionTo(new idleState);
        return true;
    }
}