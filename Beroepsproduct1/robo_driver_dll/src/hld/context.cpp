#include "context.hpp"
#include <iostream>

Context::Context(State *state, rclcpp::Logger logger) : state_(nullptr), logger_(logger)
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
        std::cout<<"test7"<<std::endl;
        delete this->state_;
        std::cout<<"test8"<<std::endl;
    }

    this->state_ = state;
    this->state_->set_context(this);

    std::cout<<"what";

    this->state_->f_entry();
    std::cout<<"done"<<std::endl;
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