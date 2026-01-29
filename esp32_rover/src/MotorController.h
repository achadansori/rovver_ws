#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

/**
 * @brief Motor Controller class for differential drive rover
 * 
 * Controls two motors (left and right) using PWM signals.
 * Supports various motor drivers (L298N, TB6612, etc.)
 */
class MotorController {
public:
    /**
     * @brief Construct a new Motor Controller
     * 
     * @param leftPwmPin PWM pin for left motor speed
     * @param leftDir1Pin Direction pin 1 for left motor
     * @param leftDir2Pin Direction pin 2 for left motor
     * @param rightPwmPin PWM pin for right motor speed
     * @param rightDir1Pin Direction pin 1 for right motor
     * @param rightDir2Pin Direction pin 2 for right motor
     */
    MotorController(
        uint8_t leftPwmPin, uint8_t leftDir1Pin, uint8_t leftDir2Pin,
        uint8_t rightPwmPin, uint8_t rightDir1Pin, uint8_t rightDir2Pin
    );

    /**
     * @brief Initialize motor pins and PWM
     */
    void begin();

    /**
     * @brief Set motor speeds from linear and angular velocity
     * 
     * @param linear Linear velocity (-1.0 to 1.0)
     * @param angular Angular velocity (-1.0 to 1.0)
     */
    void setVelocity(float linear, float angular);

    /**
     * @brief Set individual motor speeds
     * 
     * @param leftSpeed Left motor speed (-255 to 255)
     * @param rightSpeed Right motor speed (-255 to 255)
     */
    void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed);

    /**
     * @brief Stop both motors immediately
     */
    void stop();

    /**
     * @brief Get last set left motor speed
     */
    int16_t getLeftSpeed() const { return _leftSpeed; }

    /**
     * @brief Get last set right motor speed
     */
    int16_t getRightSpeed() const { return _rightSpeed; }

private:
    // Pin configuration
    uint8_t _leftPwmPin;
    uint8_t _leftDir1Pin;
    uint8_t _leftDir2Pin;
    uint8_t _rightPwmPin;
    uint8_t _rightDir1Pin;
    uint8_t _rightDir2Pin;

    // PWM channel configuration (ESP32 LEDC)
    uint8_t _leftPwmChannel;
    uint8_t _rightPwmChannel;
    static const uint32_t PWM_FREQ = 5000;
    static const uint8_t PWM_RESOLUTION = 8;

    // Current motor speeds
    int16_t _leftSpeed;
    int16_t _rightSpeed;

    // Wheel base for differential drive calculation
    float _wheelBase;

    /**
     * @brief Set single motor direction and speed
     */
    void setMotor(uint8_t pwmChannel, uint8_t dir1Pin, uint8_t dir2Pin, int16_t speed);
};

#endif // MOTOR_CONTROLLER_H
