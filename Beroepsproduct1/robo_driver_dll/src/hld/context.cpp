#include "context.hpp"
#include <iostream>

Context::Context(State *state, boost::asio::serial_port &serial, rclcpp::Logger logger) : state_(nullptr), serialPort_(serial), logger_(logger), commandQueue_()
{
    this->TransitionTo(state);
}
Context::~Context()
{
    delete state_;
}
/**
 * The Context allows changing the State object at runtime.
 */
void Context::TransitionTo(State *state)
{

    if (this->state_ != nullptr)
    {
        this->state_->f_exit();
        delete this->state_;
    }

    this->state_ = state;
    this->state_->set_context(this);

    this->state_->f_entry();
}

/**
 * The Context delegates part of its behavior to the current State object.
 */
void Context::f_entry()
{
    this->state_->f_entry();
}
void Context::f_do()
{
    this->state_->f_do();
    checkAllTriggers();
}
void Context::f_exit()
{
    this->state_->f_exit();
}

bool Context::checkAllTriggers()
{
    return this->state_->checkAllTriggers();
}