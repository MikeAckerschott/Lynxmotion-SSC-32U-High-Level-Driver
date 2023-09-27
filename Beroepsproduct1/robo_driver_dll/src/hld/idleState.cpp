#include "idleState.hpp"
#include "movingState.hpp"

void idleState::f_entry()
{
}

idleState::idleState()
{
}

void idleState::f_do()
{
    std::cout << "test" << std::endl;
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
    std::cout << "CHECKING IF PROGRAMMED POSITOIN RECEIEVED" << std::endl;
    return context_->programmedPositionCommandReceived;
}

bool idleState::checkAllTriggers()
{
    std::cout << "test2" << std::endl;
    if (emergencyStopReceived())
    {
        // TODO transition to emergency stop state
    }

    if (singleServoCommandReceived())
    {
        context_->TransitionTo(new movingState);
        context_->singleServoCommandReceived = false;
        RCLCPP_INFO(context_->logger_, "SingleServoCommand received. IdleState exit. MovingState entry");
        return true;
    }

    std::cout << "test3" << std::endl;

    if (multiServoCommandReceived())
    {
        context_->TransitionTo(new movingState);
        context_->multiServoCommandReceived = false;
        return true;
    }
    std::cout << "test4" << std::endl;

    if (programmedPositionCommandReceived())
    {
        std::cout << "programmedPositionReceived" << std::endl;
        context_->programmedPositionCommandReceived = false;
        context_->TransitionTo(new movingState);

        // RCLCPP_INFO(context_->logger_, "programmedPosition received. IdleState exit. MovingState entry");

        std::cout << "return true" << std::endl;

        return true;
    }
    std::cout << "test5" << std::endl;

    return false;
}