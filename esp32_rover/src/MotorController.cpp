#include "MotorController.h"

MotorController::MotorController(
    uint8_t leftPwmPin, uint8_t leftDir1Pin, uint8_t leftDir2Pin,
    uint8_t rightPwmPin, uint8_t rightDir1Pin, uint8_t rightDir2Pin
) : _leftPwmPin(leftPwmPin),
    _leftDir1Pin(leftDir1Pin),
    _leftDir2Pin(leftDir2Pin),
    _rightPwmPin(rightPwmPin),
    _rightDir1Pin(rightDir1Pin),
    _rightDir2Pin(rightDir2Pin),
    _leftPwmChannel(0),
    _rightPwmChannel(1),
    _leftSpeed(0),
    _rightSpeed(0),
    _wheelBase(0.3f)  // Default wheel base 30cm
{
}

void MotorController::begin() {
    // Configure direction pins as outputs
    pinMode(_leftDir1Pin, OUTPUT);
    pinMode(_leftDir2Pin, OUTPUT);
    pinMode(_rightDir1Pin, OUTPUT);
    pinMode(_rightDir2Pin, OUTPUT);

    // Configure PWM channels for ESP32
    ledcSetup(_leftPwmChannel, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(_rightPwmChannel, PWM_FREQ, PWM_RESOLUTION);

    // Attach PWM channels to pins
    ledcAttachPin(_leftPwmPin, _leftPwmChannel);
    ledcAttachPin(_rightPwmPin, _rightPwmChannel);

    // Ensure motors are stopped
    stop();

    Serial.println("[MotorController] Initialized");
    Serial.printf("  Left: PWM=%d, Dir1=%d, Dir2=%d\n", 
                  _leftPwmPin, _leftDir1Pin, _leftDir2Pin);
    Serial.printf("  Right: PWM=%d, Dir1=%d, Dir2=%d\n", 
                  _rightPwmPin, _rightDir1Pin, _rightDir2Pin);
}

void MotorController::setVelocity(float linear, float angular) {
    // Differential drive kinematics
    // Convert linear (m/s) and angular (rad/s) to wheel speeds
    
    // Normalize inputs to -1.0 to 1.0 range
    linear = constrain(linear, -1.0f, 1.0f);
    angular = constrain(angular, -1.0f, 1.0f);
    
    // Calculate differential speeds
    // left = linear - angular (positive angular = turn left = slow left wheel)
    // right = linear + angular
    float leftNorm = linear - angular;
    float rightNorm = linear + angular;
    
    // Scale to prevent overflow (if both are > 1.0)
    float maxMag = max(abs(leftNorm), abs(rightNorm));
    if (maxMag > 1.0f) {
        leftNorm /= maxMag;
        rightNorm /= maxMag;
    }
    
    // Convert to PWM range (-255 to 255)
    int16_t leftSpeed = (int16_t)(leftNorm * 255.0f);
    int16_t rightSpeed = (int16_t)(rightNorm * 255.0f);
    
    setMotorSpeeds(leftSpeed, rightSpeed);
}

void MotorController::setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    _leftSpeed = constrain(leftSpeed, -255, 255);
    _rightSpeed = constrain(rightSpeed, -255, 255);
    
    setMotor(_leftPwmChannel, _leftDir1Pin, _leftDir2Pin, _leftSpeed);
    setMotor(_rightPwmChannel, _rightDir1Pin, _rightDir2Pin, _rightSpeed);
    
    Serial.printf("[Motor] L:%4d R:%4d\n", _leftSpeed, _rightSpeed);
}

void MotorController::setMotor(uint8_t pwmChannel, uint8_t dir1Pin, uint8_t dir2Pin, int16_t speed) {
    if (speed >= 0) {
        // Forward direction
        digitalWrite(dir1Pin, HIGH);
        digitalWrite(dir2Pin, LOW);
        ledcWrite(pwmChannel, speed);
    } else {
        // Reverse direction
        digitalWrite(dir1Pin, LOW);
        digitalWrite(dir2Pin, HIGH);
        ledcWrite(pwmChannel, -speed);
    }
}

void MotorController::stop() {
    _leftSpeed = 0;
    _rightSpeed = 0;
    
    // Set all direction pins LOW
    digitalWrite(_leftDir1Pin, LOW);
    digitalWrite(_leftDir2Pin, LOW);
    digitalWrite(_rightDir1Pin, LOW);
    digitalWrite(_rightDir2Pin, LOW);
    
    // Set PWM to 0
    ledcWrite(_leftPwmChannel, 0);
    ledcWrite(_rightPwmChannel, 0);
    
    Serial.println("[Motor] STOPPED");
}
