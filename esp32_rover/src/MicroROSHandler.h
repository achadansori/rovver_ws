#ifndef MICRO_ROS_HANDLER_H
#define MICRO_ROS_HANDLER_H

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include "MotorController.h"

/**
 * @brief micro-ROS Handler class
 * 
 * Manages WiFi connection and micro-ROS communication.
 * Subscribes to cmd_vel and forwards commands to MotorController.
 */
class MicroROSHandler {
public:
    /**
     * @brief Construct a new MicroROS Handler
     * 
     * @param motorController Reference to motor controller
     */
    MicroROSHandler(MotorController& motorController);

    /**
     * @brief Destructor - cleanup ROS resources
     */
    ~MicroROSHandler();

    /**
     * @brief Initialize WiFi and micro-ROS
     * 
     * @param ssid WiFi SSID
     * @param password WiFi password
     * @param agentIp micro-ROS agent IP address
     * @param agentPort micro-ROS agent port
     * @return true if initialization successful
     */
    bool begin(const char* ssid, const char* password, 
               const char* agentIp, uint16_t agentPort);

    /**
     * @brief Spin the executor (call in loop)
     */
    void spin();

    /**
     * @brief Check if connected to agent
     */
    bool isConnected() const { return _connected; }

    /**
     * @brief Handle connection loss
     */
    void handleDisconnect();

    /**
     * @brief Attempt to reconnect to agent
     */
    bool reconnect();

private:
    // Motor controller reference
    MotorController& _motorController;

    // micro-ROS entities
    rcl_allocator_t _allocator;
    rclc_support_t _support;
    rcl_node_t _node;
    rcl_subscription_t _cmdVelSub;
    rclc_executor_t _executor;
    
    // Message buffer
    geometry_msgs__msg__Twist _twistMsg;

    // Connection state
    bool _connected;
    unsigned long _lastMsgTime;
    static const unsigned long TIMEOUT_MS = 1000; // Watchdog timeout

    // WiFi credentials storage
    const char* _ssid;
    const char* _password;
    const char* _agentIp;
    uint16_t _agentPort;

    /**
     * @brief Static callback wrapper for cmd_vel subscription
     */
    static void cmdVelCallback(const void* msgIn);

    /**
     * @brief Instance callback for cmd_vel
     */
    void handleCmdVel(const geometry_msgs__msg__Twist* msg);

    // Singleton for static callback access
    static MicroROSHandler* _instance;
};

#endif // MICRO_ROS_HANDLER_H
