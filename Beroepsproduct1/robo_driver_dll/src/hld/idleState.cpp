#include "highLevelNode.hpp"
#include "idleState.hpp"
#include "movingState.hpp"

void idleState::f_entry()
{
    std::cout<<" IDLE ENTRY"<<std::endl;
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
    return context_->emergencyStopReceived;
}

bool idleState::programmedPositionCommandReceived()
{
    return context_->programmedPositionCommandReceived;
}

bool idleState::checkAllTriggers()
{
    if (emergencyStopReceived())
    {
        // TODO transition to emergency stop state
    }

    if (singleServoCommandReceived())
    {
        context_->TransitionTo(new movingState);
        context_->singleServoCommandReceived = false;
        RCLCPP_INFO(context_->node_->get_logger(), "SingleServoCommand received. IdleState exit. MovingState entry");
        return true;
    }

    if (multiServoCommandReceived())
    {
        context_->TransitionTo(new movingState);
        context_->multiServoCommandReceived = false;
        RCLCPP_INFO(context_->node_->get_logger(), "mutliServoCommand received. IdleState exit. MovingState entry");
        return true;
    }

    if (programmedPositionCommandReceived())
    {
        std::cout << "programmedPositionReceived" << std::endl;
        context_->programmedPositionCommandReceived = false;
        RCLCPP_INFO(context_->node_->get_logger(), "programmedPositionCommand received. IdleState exit. MovingState entry");
        context_->TransitionTo(new movingState);

        return true;
    }

    return false;
}