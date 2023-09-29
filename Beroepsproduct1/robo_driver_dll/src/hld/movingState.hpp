#ifndef MOVINGSTATE_HPP_
#define MOVINGSTATE_HPP_

/**
 * @file movingState.hpp
 * @brief Contains the movingState class
 * @details The movingState class is used to handle the moving state of the robotic arm
*/

#include "state.hpp"
#include "../lld/command.hpp"
#include "idleState.hpp"
#include "emergencyStopState.hpp"
#include "../lld/lowLevelServer.hpp"

/**
 * @class movingState
 * @brief The movingState class is used to handle the moving state of the robotic arm
*/
class movingState : public State
{
public:
    /**
     * @brief Constructor
     */
    movingState();

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
     * @brief A trigger that checks if the current movement is done
     * @return True if the movement is done, false if not
     */
    bool isMovingDone();

    /**
     * @brief A trigger that checks if a new position command has been received
     * @return True if a new position command has been received, false if not
     */
    bool newPositionCommandReceived();

    /**
     * @brief A trigger that checks if a new single servo command has been received
     * @return True if a new single servo command has been received, false if not
     */
    bool newSingleServoCommandReceived();

    /**
     * @brief A trigger that checks if a new multi servo command has been received
     * @return True if a new multi servo command has been received, false if not
     */
    bool newMultiServoCommandReceived();

    /**
     * @brief A trigger that checks if the commandQueue is empty
     * @return True if the commandQueue is empty, false if not
     */
    bool isQueueEmpty();

    /**
     * @brief A trigger that checks if an emergency stop has been received
     * @return True if an emergency stop has been received, false if not
     */
    bool emergencyStopReceived();

private:
};

#endif /*MOVINGSTATE_HPP_*/