#ifndef IDLESTATE_HPP_
#define IDLESTATE_HPP_

/**
 * @file idleState.hpp
 * @brief Contains the idleState class
 * @details The idleState class is used to handle the idle state of the robotic arm
 */

#include "state.hpp"
#include <iostream>

/**
 * @class idleState
 * @brief The idleState class is used to handle the idle state of the robotic arm
 */
class idleState : public State
{
public:
    /**
     * @brief Constructor
     */
    idleState();

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
     * @brief A trigger that checks if a single servo command has been received
     * @return True if the state should be exited, false if not
     */
    bool singleServoCommandReceived();

    /**
     * @brief A trigger that checks if a multi servo command has been received
     * @return True if the state should be exited, false if not
     */
    bool multiServoCommandReceived();

    /**
     * @brief A trigger that checks if an emergency stop has been received
     * @return True if the state should be exited, false if not
     */
    bool emergencyStopReceived();

    /**
     * @brief A trigger that checks if a programmed position command has been received
     * @return True if the state should be exited, false if not
     */
    bool programmedPositionCommandReceived();

    /**
     * @brief A trigger that checks if the movement queue is not empty
     * @return True if the state should be exited, false if not
     */
    bool movementQueueNotEmpty();
};

#endif /*IDLESTATE_HPP_*/