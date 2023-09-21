#ifndef COMMUNICATORNODE_HPP_
#define COMMUNICATORNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "msg_srv/srv/single_servo_command.hpp"
#include "msg_srv/srv/multi_servo_command.hpp"
#include "msg_srv/srv/emergency_stop.hpp"
#include "msg_srv/srv/move_to_position.hpp"

class CommunicatorNode : public rclcpp::Node
{

public:
    CommunicatorNode();
    ~CommunicatorNode();

    /**
   * @brief Sends a singleServoCommand to the low level driver
   * @param servo The servo to move
   * @param angle The angle to move to
   * @param speed The speed to move at in in microseconds per second (See Lynxmotion SSC-32U USB user guide page 24)
   * @param time  The time to move in milliseconds
   */
  void sendSingleServoCommand(short servo, unsigned long long angle, unsigned long long movement, std::string movementType);

  /**
   * @brief Sends a multiServoCommand to the low level driver
   * @param arguments A vector of vectors containing the servo, angle, speed and time for each servo
   */
  void sendMultiServoCommand(std::vector<std::vector<long long>> arguments, std::vector<std::string> movementTypes);

  /**
   * @brief Sends a stopCommand to the low level driver
   */
  void sendStopCommand();

  /**
   * @brief Sends a programmedPositionCommand to the low level driver
   * @param programmedPosition The programmed position to move to (park, ready, straight-up)
   */
  void sendProgrammedPositionCommand(std::string programmedPosition);

private:
    /**
     * @brief Client to send singleServoCommand to the low level driver.
     * Expects a ServoCommand message as input, which is representable as follows:
     * {
     *  uint8 target_servo
     *  int64 position
     *  int64 duration
     *  int64 speed
     * }
     *
     * @returns true if the command was validated by the low level driver, false if not
     */
    rclcpp::Client<msg_srv::srv::SingleServoCommand>::SharedPtr singleServoClient_;

    /**
     * @brief Client to send multiServoCommand to the low level driver.
     * Expects a Move message as input, which is representable as follows:
     * {
     * ServoCommand[]
     * }
     *
     * A ServoCommand is representable as follows:
     * {
     * uint8 target_servo
     * int64 position
     * int64 duration
     * int64 speed
     * }
     *
     * @returns true if the command was validated by the low level driver, false if not
     */
    rclcpp::Client<msg_srv::srv::MultiServoCommand>::SharedPtr multiServoClient_;

    /**
     * @brief Client to send stopCommand to the low level driver.
     * Expects an empty message as input.
     *
     * @returns true if the command was validated by the low level driver, false if not
     */
    rclcpp::Client<msg_srv::srv::EmergencyStop>::SharedPtr stopClient_;

    /**
     * @brief Client to send programmedPositionCommand to the low level driver.
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
     * If the input is not one of the above, the command will not be validated by the low level driver.
     *
     * @returns true if the command was validated by the low level driver, false if not
     */
    rclcpp::Client<msg_srv::srv::MoveToPosition>::SharedPtr programmedPositionClient_;
};

#endif /* COMMUNICATORNODE_HPP_ */