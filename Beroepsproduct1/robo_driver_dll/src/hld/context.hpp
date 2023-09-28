#ifndef CONTEXT_HPP_
#define CONTEXT_HPP_

#include "state.hpp"
#include <queue>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include "../lld/command.hpp"

// #include "highLevelNode.hpp"

class State;

class HighLevelNode;

class Context
{

private:
    State *state_;

public:
    Context(State *state, boost::asio::serial_port &serial, rclcpp::Logger logger);
    ~Context();

    void TransitionTo(State *state);

    void f_entry();
    void f_do();
    void f_exit();

    bool checkAllTriggers();

    bool singleServoCommandReceived;
    bool multiServoCommandReceived;
    bool programmedPositionCommandReceived;
    bool emergencyStopDeactivateRequest;
    bool emergencyStopActivateRequest;
    bool skipCommandReceived;

    boost::asio::serial_port &serialPort_;
    rclcpp::Logger logger_;

    std::queue<Command> commandQueue_;
};

#endif /* CONTEXT_HPP_ */