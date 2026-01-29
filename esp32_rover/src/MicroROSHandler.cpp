#include "MicroROSHandler.h"
#include <WiFi.h>

// Static instance for callback access
MicroROSHandler* MicroROSHandler::_instance = nullptr;

MicroROSHandler::MicroROSHandler(MotorController& motorController)
    : _motorController(motorController),
      _connected(false),
      _lastMsgTime(0),
      _ssid(nullptr),
      _password(nullptr),
      _agentIp(nullptr),
      _agentPort(0)
{
    _instance = this;
}

MicroROSHandler::~MicroROSHandler() {
    if (_connected) {
        rcl_subscription_fini(&_cmdVelSub, &_node);
        rcl_node_fini(&_node);
        rclc_support_fini(&_support);
    }
    _instance = nullptr;
}

bool MicroROSHandler::begin(const char* ssid, const char* password,
                            const char* agentIp, uint16_t agentPort) {
    _ssid = ssid;
    _password = password;
    _agentIp = agentIp;
    _agentPort = agentPort;

    Serial.println("[MicroROS] Starting WiFi Access Point...");
    Serial.printf("  SSID: %s\n", ssid);
    Serial.printf("  Password: %s\n", password);

    // Start Access Point mode
    WiFi.mode(WIFI_AP);
    bool apStarted = WiFi.softAP(ssid, password);
    
    if (!apStarted) {
        Serial.println("[MicroROS] Failed to start Access Point!");
        return false;
    }
    
    // Set static IP for AP (default is 192.168.4.1)
    delay(100);
    
    Serial.println("[MicroROS] Access Point Started!");
    Serial.printf("  AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.println("  >> Connect your computer to this WiFi network <<");
    Serial.printf("  >> Then run micro-ROS agent, ESP32 will connect to: %s:%d <<\n", agentIp, agentPort);

    // Configure micro-ROS transport
    Serial.printf("[MicroROS] Connecting to agent at %s:%d\n", agentIp, agentPort);
    
    IPAddress agent_ip;
    agent_ip.fromString(agentIp);
    set_microros_wifi_transports(
        const_cast<char*>(ssid),
        const_cast<char*>(password),
        agent_ip,
        agentPort
    );

    // Wait for agent connection
    delay(2000);

    // Initialize allocator
    _allocator = rcl_get_default_allocator();

    // Initialize support
    rcl_ret_t ret = rclc_support_init(&_support, 0, NULL, &_allocator);
    if (ret != RCL_RET_OK) {
        Serial.printf("[MicroROS] Support init failed: %d\n", (int)ret);
        return false;
    }

    // Create node
    ret = rclc_node_init_default(&_node, "esp32_rover", "", &_support);
    if (ret != RCL_RET_OK) {
        Serial.printf("[MicroROS] Node init failed: %d\n", (int)ret);
        return false;
    }
    Serial.println("[MicroROS] Node created: esp32_rover");

    // Create subscriber for cmd_vel
    ret = rclc_subscription_init_default(
        &_cmdVelSub,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );
    if (ret != RCL_RET_OK) {
        Serial.printf("[MicroROS] Subscription init failed: %d\n", (int)ret);
        return false;
    }
    Serial.println("[MicroROS] Subscribed to: /cmd_vel");

    // Create executor
    ret = rclc_executor_init(&_executor, &_support.context, 1, &_allocator);
    if (ret != RCL_RET_OK) {
        Serial.printf("[MicroROS] Executor init failed: %d\n", (int)ret);
        return false;
    }

    // Add subscription to executor
    ret = rclc_executor_add_subscription(
        &_executor,
        &_cmdVelSub,
        &_twistMsg,
        &MicroROSHandler::cmdVelCallback,
        ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) {
        Serial.printf("[MicroROS] Add subscription failed: %d\n", (int)ret);
        return false;
    }

    _connected = true;
    _lastMsgTime = millis();
    
    Serial.println("[MicroROS] Initialization complete!");
    return true;
}

void MicroROSHandler::spin() {
    if (!_connected) {
        return;
    }

    // Spin executor with timeout
    rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(10));

    // Watchdog - stop motors if no message received
    if (millis() - _lastMsgTime > TIMEOUT_MS) {
        _motorController.stop();
    }
}

void MicroROSHandler::cmdVelCallback(const void* msgIn) {
    if (_instance != nullptr) {
        const geometry_msgs__msg__Twist* msg = 
            static_cast<const geometry_msgs__msg__Twist*>(msgIn);
        _instance->handleCmdVel(msg);
    }
}

void MicroROSHandler::handleCmdVel(const geometry_msgs__msg__Twist* msg) {
    _lastMsgTime = millis();
    
    // Extract linear.x and angular.z
    float linear = msg->linear.x;
    float angular = msg->angular.z;
    
    Serial.printf("[CmdVel] Linear: %.2f, Angular: %.2f\n", linear, angular);
    
    // Send to motor controller
    _motorController.setVelocity(linear, angular);
}

void MicroROSHandler::handleDisconnect() {
    _connected = false;
    _motorController.stop();
    Serial.println("[MicroROS] Disconnected!");
}

bool MicroROSHandler::reconnect() {
    Serial.println("[MicroROS] Attempting reconnection...");
    
    // Cleanup old resources
    if (_connected) {
        rclc_executor_fini(&_executor);
        rcl_subscription_fini(&_cmdVelSub, &_node);
        rcl_node_fini(&_node);
        rclc_support_fini(&_support);
    }
    
    _connected = false;
    
    // Try to reconnect
    return begin(_ssid, _password, _agentIp, _agentPort);
}
