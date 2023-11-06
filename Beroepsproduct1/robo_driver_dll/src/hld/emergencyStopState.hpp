#ifndef EMERGENCYSTOPSTATE_HPP_
#define EMERGENCYSTOPSTATE_HPP_

/**
 * @file emergencyStopState.hpp
 * @brief Contains the emergencyStopState class
 * @details The emergencyStopState class is used to handle the emergency stop state of the robotic arm
 */

#include "state.hpp"
#include "../lld/command.hpp"
#include "idleState.hpp"

/**
 * @class emergencyStopState
 * @brief The emergencyStopState class is used to handle the emergency stop state of the robotic arm
 */
class emergencyStopState : public State
{
public:
    /**
     * @brief Constructor
     */
    emergencyStopState();

    /**
     * @brief function that is called when the state is entered
     */
    void f_entry() override;

    /**
     * @brief function that is called as long as the state is active
     */
    void f_do() override;

    /**
     * @brief function that is called when the state is exited
     */
    void f_exit() override;

    /**
     * @brief Checks if any trigger is activated
     * @return True if the state should be exited, false if not
     */
    bool checkAllTriggers() override;

    /**
     * @brief A trigger that checks if the emergency stop has been deactivated
     * @return True if the state should be exited, false if not
     */
    bool deactivateEmergencyStopReceived();

private:
};

#endif /*EMERGENCYSTOPSTATE_HPP_*/