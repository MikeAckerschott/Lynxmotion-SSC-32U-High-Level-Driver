#ifndef IDLESTATE_HPP_
#define IDLESTATE_HPP_

#include "state.hpp"
#include <iostream>

class idleState : public State
{
public:
    idleState();

    void f_entry() override;
    void f_do() override;
    void f_exit() override;

    bool checkAllTriggers() override;

    bool singleServoCommandReceived();
    bool multiServoCommandReceived();
    bool emergencyStopReceived();
    bool programmedPositionCommandReceived();
};

#endif /*IDLESTATE_HPP_*/