#ifndef CONTEXT_HPP_
#define CONTEXT_HPP_

/**
 * @file context.hpp
 * @brief Contains the Context class that is used to transition between states
 * @details The Context class is used to transition between states. It also contains the serial port and logger that is used by the states
 */

#include "state.hpp"
#include <queue>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include "../lld/command.hpp"

class State;

/**
 * @class Context
 * @brief Contains the Context class that is used to transition between states
 * @details The Context class is used to transition between states. It also contains the serial port and logger that is used by the states
 */

class Context
{

private:

    /**
     * @brief The current state
     */

    State *state_;

public:
    /**
     * @brief Constructor
     * @param state The initial state
     * @param serial The serial port
     * @param logger The logger
     */
    Context(State *state, boost::asio::serial_port &serial, rclcpp::Logger logger);

    /**
     * @brief Destructor
     */
    ~Context();

    /**
     * @brief Transitions to a new state
     * @param state The new state
     */
    void TransitionTo(State *state);

    /**
     * @brief Calls the entry function of the current state
     */
    void f_entry();

    /**
     * @brief Calls the do function of the current state
     */
    void f_do();

    /**
     * @brief Calls the exit function of the current state
     */
    void f_exit();

    /**
     * @brief Calls the checkAllTriggers function of the current state
     * @return True if a trigger was activated, false if not
     */
    bool checkAllTriggers();

    /**
     * @brief variable to check if a singleServoCommand was received
    */
    bool singleServoCommandReceived;

    /**
     * @brief variable to check if a multiServoCommand was received
    */
    bool multiServoCommandReceived;

    /**
     * @brief variable to check if a programmedPositionCommand was received
    */
    bool programmedPositionCommandReceived;

    /**
     * @brief variable to check if a emergencyStopDeactivateRequest was received
    */
    bool emergencyStopDeactivateRequest;

    /**
     * @brief variable to check if a emergencyStopActivateRequest was received
    */
    bool emergencyStopActivateRequest;

    /**
     * @brief variable to check if a skipCommand was received
    */
    bool skipCommandReceived;

    /**
     * @brief variable to check if a emptyQueueCommand was received
    */
    bool emptyQueueCommandReceived;

    /**
     * @brief the serial port over which the commands are sent to the robotic arm
    */
    boost::asio::serial_port &serialPort_;

    /**
     * @brief the logger used to log messages. received from the HighLevelNode
    */
    rclcpp::Logger logger_;

    /**
     * @brief the queue of commands to execute
    */
    std::queue<Command> commandQueue_;
};

#endif /* CONTEXT_HPP_ */