#ifndef CONTEXT_HPP_
#define CONTEXT_HPP_

#include "state.hpp"
#include <deque>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
// #include "highLevelNode.hpp"

class State;

class HighLevelNode;

class Context
{

    /**
     * The Context defines the interface of interest to clients. It also maintains a
     * reference to an instance of a State subclass, which represents the current
     * state of the Context.
     */
    /**
     * @var State A reference to the current state of the Context.
     */
private:
    State *state_;

    // HighLevelNode *node;

public:
    Context(State *state, boost::asio::serial_port &serial, rclcpp::Logger logger);
    ~Context();
    /**
     * The Context allows changing the State object at runtime.
     */
    void TransitionTo(State *state);
    /**
     * The Context delegates part of its behavior to the current State object.
     */
    void f_entry();
    void f_do();
    void f_exit();

    bool checkAllTriggers();

    bool singleServoCommandReceived;
    bool multiServoCommandReceived;
    bool programmedPositionCommandReceived;
    bool emergencyStopReceived;


    boost::asio::serial_port &serialPort_;
    rclcpp::Logger logger_;
};

#endif /* CONTEXT_HPP_ */