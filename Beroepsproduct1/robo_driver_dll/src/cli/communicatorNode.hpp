#ifndef COMMUNICATORNODE_HPP_
#define COMMUNICATORNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "msg_srv/srv/single_servo_command.hpp"
#include "msg_srv/srv/multi_servo_command.hpp"
#include "msg_srv/srv/emergency_stop.hpp"
#include "msg_srv/srv/move_to_position.hpp"
#include "msg_srv/srv/skip.hpp"
#include "msg_srv/srv/empty_queue.hpp"

/**
 * @class CommunicatorNode
 * @brief Node that sends commands to the high level driver
 * @details Currently supports commands: singleServo, multiServo, stop, programmedPosition
 */

class CommunicatorNode : public rclcpp::Node
{

public:
    /**
     * @brief Constructor
     */
    CommunicatorNode();

    /**
     * @brief Destructor
     */
    ~CommunicatorNode();

    /**
     * @brief Sends a singleServoCommand to the high level driver
     * @param servo The servo to move
     * @param angle The angle to move to
     * @param movement the speed or duration value depending on the movementType
     * @param movementType  move the servo with speed or duration. (example: T1000 or S1000)
     */
    void sendSingleServoCommand(int servo, int angle, int movement, std::string movementType);

    /**
     * @brief Sends a multiServoCommand to the high level driver
     * @param arguments A vector of vectors containing the servo, angle and movement values
     * @param movementTypes A vector of strings containing the movement types for each servo (speed or duration)
     */
    void sendMultiServoCommand(std::vector<std::vector<long long>> arguments, std::vector<std::string> movementTypes);

    /**
     * @brief Sends a stopCommand to the high level driver
     * @details This will enable the emergency stop
     */
    void sendStopCommand();

    /**
     * @brief sends an emergencyStop to the high level driver with param enable to false
     * @details This will disable the emergency stop
     */

    void deactivateEmergencyStop();

    /**
     * @brief Sends a programmedPositionCommand to the high level driver
     * @param programmedPosition The programmed position to move to (park, ready, straight-up)
     */
    void sendProgrammedPositionCommand(std::string programmedPosition);

    /**
     * @brief Sends a skipCommand to the high level driver
     * @details This will skip the current command in the queue
     */

    void sendSkipCommand();

    /**
     * @brief Sends an emptyQueueCommand to the high level driver
     * @details This will empty the commandqueue on the HLD
     */

    void sendEmptyQueueCommand();

private:
    /**
     * @brief Client to send singleServoCommand to the high level driver.
     * Expects a ServoCommand message as input, which is representable as follows:
     * {
     * uint8 target_servo
     * int64 degrees
     * int64 movement
     * string movement_type
     * }
     *
     * @returns true if the command was validated by the high level driver, false if not
     */
    rclcpp::Client<msg_srv::srv::SingleServoCommand>::SharedPtr singleServoClient_;

    /**
     * @brief Client to send multiServoCommand to the high level driver.
     * Expects a Move message as input, which is representable as follows:
     * {
     * ServoCommand[]
     * }
     *
     * A ServoCommand is representable as follows:
     * {
     * uint8 target_servo
     * int64 degrees
     * int64 movement
     * string movement_type
     * }
     *
     * @returns true if the command was validated by the high level driver, false if not
     */
    rclcpp::Client<msg_srv::srv::MultiServoCommand>::SharedPtr multiServoClient_;

    /**
     * @brief Client to send stopCommand to the high level driver.
     * Expects an empty message as input.
     *
     * @returns true if the command was validated by the high level driver, false if not
     */
    rclcpp::Client<msg_srv::srv::EmergencyStop>::SharedPtr stopClient_;

    /**
     * @brief Client to send programmedPositionCommand to the high level driver.
     * Expects a ProgrammedPosition message as input, which is representable as follows:
     * {
     * string programmed_position
     * }
     *
     * programmedPosition can be one of the following:
     * "park"
     * "ready"
     * "straight-up"
     *
     * If the input is not one of the above, the command will not be validated by the high level driver.
     *
     * @returns true if the command was validated by the high level driver, false if not
     */
    rclcpp::Client<msg_srv::srv::MoveToPosition>::SharedPtr programmedPositionClient_;

    /**
     * @brief Client to send skipCommand to the high level driver.
     * Expects an empty message as input.
     *
     * @returns a boolean called empty, which is true if the queue was empty at call of this function, false if not
     */

    rclcpp::Client<msg_srv::srv::Skip>::SharedPtr skipClient_;

    /**
     * @brief Client to send emptyQueueCommand to the high level driver.
     * Expects an empty message as input.
     *
     * @returns a boolean called empty, which is true if the queue was empty at call of this function, false if not
     */

    rclcpp::Client<msg_srv::srv::EmptyQueue>::SharedPtr emptyQueueClient_;
};

#endif /* COMMUNICATORNODE_HPP_ */