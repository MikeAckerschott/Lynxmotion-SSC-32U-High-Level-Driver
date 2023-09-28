#ifndef EMERGENCYSTOPSTATE_HPP_
#define EMERGENCYSTOPSTATE_HPP_

#include "state.hpp"
#include "../lld/command.hpp"
#include "idleState.hpp"

class emergencyStopState : public State
{
public:
    emergencyStopState();

    void f_entry() override;
    void f_do() override;
    void f_exit() override;

    bool checkAllTriggers() override;

    bool deactivateEmergencyStopReceived();

private:
};

#endif /*EMERGENCYSTOPSTATE_HPP_*/