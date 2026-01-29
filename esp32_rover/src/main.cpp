/**
 * ESP32 Rover - micro-ROS Firmware
 * 
 * Receives cmd_vel commands via micro-ROS over WiFi
 * and controls differential drive motors.
 * 
 * Author: User
 * Date: 2026-01-28
 */

#include <Arduino.h>
#include "MotorController.h"
#include "MicroROSHandler.h"

// ============================================
// PIN CONFIGURATION - ADJUST FOR YOUR SETUP!
// ============================================

// Motor Driver Pins (e.g., for L298N)
// Left Motor
#define LEFT_PWM_PIN    25  // ENA on L298N
#define LEFT_DIR1_PIN   26  // IN1 on L298N
#define LEFT_DIR2_PIN   27  // IN2 on L298N

// Right Motor
#define RIGHT_PWM_PIN   14  // ENB on L298N
#define RIGHT_DIR1_PIN  12  // IN3 on L298N
#define RIGHT_DIR2_PIN  13  // IN4 on L298N

// ============================================
// WIFI ACCESS POINT CONFIGURATION
// ============================================

// AP credentials (can be overridden via build flags)
#ifndef AP_SSID
#define AP_SSID "RoverESP32"
#endif

#ifndef AP_PASSWORD
#define AP_PASSWORD "rover1234"
#endif

// micro-ROS Agent (run on your PC)
#ifndef AGENT_IP
#define AGENT_IP "192.168.1.100"
#endif

#ifndef AGENT_PORT
#define AGENT_PORT 8888
#endif

// ============================================
// GLOBAL OBJECTS
// ============================================

// Create motor controller
MotorController motorController(
    LEFT_PWM_PIN, LEFT_DIR1_PIN, LEFT_DIR2_PIN,
    RIGHT_PWM_PIN, RIGHT_DIR1_PIN, RIGHT_DIR2_PIN
);

// Create micro-ROS handler
MicroROSHandler rosHandler(motorController);

// Status LED
#define STATUS_LED 2  // Built-in LED on most ESP32 boards

// ============================================
// SETUP
// ============================================

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(1000);
    
    Serial.println();
    Serial.println("========================================");
    Serial.println("   ESP32 Rover - micro-ROS Firmware");
    Serial.println("========================================");
    Serial.println();

    // Status LED
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    // Initialize motor controller
    motorController.begin();
    
    // Quick motor test (optional)
    Serial.println("[Setup] Motor test...");
    motorController.setVelocity(0.3, 0);
    delay(200);
    motorController.stop();
    delay(200);
    motorController.setVelocity(-0.3, 0);
    delay(200);
    motorController.stop();
    Serial.println("[Setup] Motor test complete");

    // Initialize micro-ROS (WiFi AP + Agent)
    if (rosHandler.begin(AP_SSID, AP_PASSWORD, AGENT_IP, AGENT_PORT)) {
        Serial.println("[Setup] micro-ROS initialized successfully!");
        digitalWrite(STATUS_LED, HIGH);
    } else {
        Serial.println("[Setup] micro-ROS initialization FAILED!");
        Serial.println("[Setup] Running in offline mode - motors stopped");
        // Blink LED to indicate error
        for (int i = 0; i < 10; i++) {
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
            delay(200);
        }
    }
    
    Serial.println();
    Serial.println("[Setup] Complete! Waiting for commands...");
    Serial.println();
}

// ============================================
// MAIN LOOP
// ============================================

unsigned long lastStatusTime = 0;
const unsigned long STATUS_INTERVAL = 5000; // Print status every 5 seconds

void loop() {
    // Spin micro-ROS executor
    rosHandler.spin();

    // Check connection status periodically
    if (millis() - lastStatusTime > STATUS_INTERVAL) {
        lastStatusTime = millis();
        
        if (rosHandler.isConnected()) {
            Serial.printf("[Status] Connected | L:%d R:%d\n", 
                         motorController.getLeftSpeed(),
                         motorController.getRightSpeed());
            digitalWrite(STATUS_LED, HIGH);
        } else {
            Serial.println("[Status] Disconnected - attempting reconnect...");
            digitalWrite(STATUS_LED, LOW);
            rosHandler.reconnect();
        }
    }

    // Small delay to prevent watchdog issues
    delay(1);
}
