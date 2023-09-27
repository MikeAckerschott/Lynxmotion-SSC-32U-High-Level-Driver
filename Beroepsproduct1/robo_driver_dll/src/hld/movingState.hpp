#ifndef MOVINGSTATE_HPP_
#define MOVINGSTATE_HPP_

#include "state.hpp"
#include "../lld/command.hpp"
#include "idleState.hpp"

class movingState : public State
{
public:
    movingState();

    void f_entry() override;
    void f_do() override;
    void f_exit() override;

    bool checkAllTriggers() override;

    bool isMovingDone();
    bool emergencyStopReceived();

private:
};

#endif /*MOVINGSTATE_HPP_*/