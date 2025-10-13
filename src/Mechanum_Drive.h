// #ifndef Mechanum_Drive_H
// #define Mechanum_Drive_H

// #pragma once

#include <Arduino.h>

// #if __cplusplus < 201103L
// #error "This library requires C++11 or higher"
// #endif

// Motor class for controlling individual motors
class Motor
{
public:
    Motor(int pinIn1, int pinIn2, int pinPWM, int offset)
        : pinIn1(pinIn1), pinIn2(pinIn2), pinPWM(pinPWM), speedOffset(offset)
    {
        pinMode(pinIn1, OUTPUT);
        pinMode(pinIn2, OUTPUT);
        pinMode(pinPWM, OUTPUT);
    }

    void drive(int speed)
    {
        speed = speed * speedOffset;
        if (speed >= 0)
            moveForward(speed);
        else
            moveReverse(-speed);
    }

    void brake()
    {
        digitalWrite(pinIn1, HIGH);
        digitalWrite(pinIn2, HIGH);
        analogWrite(pinPWM, 0);
    }

private:
    int pinIn1, pinIn2, pinPWM, speedOffset;

    inline void moveForward(int speed)
    {
        digitalWrite(pinIn1, HIGH);
        digitalWrite(pinIn2, LOW);
        analogWrite(pinPWM, speed);
    }

    inline void moveReverse(int speed)
    {
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, HIGH);
        analogWrite(pinPWM, speed);
    }
};

// Brake all motors
inline void brakeAll(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4)
{
    motor1.brake();
    motor2.brake();
    motor3.brake();
    motor4.brake();
}

// Move robot in clockwise direction (rotation)
inline void rotateClockwise(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4, int speed)
{
    motor1.drive(-speed);
    motor2.drive(speed);
    motor3.drive(-speed);
    motor4.drive(speed);
}

// Move robot in counter-clockwise direction (rotation)
inline void rotateCounterClockwise(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4, int speed)
{
    motor1.drive(speed);
    motor2.drive(-speed);
    motor3.drive(speed);
    motor4.drive(-speed);
}

// Move robot to the left (sideways movement)
inline void moveLeft(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4, int speed)
{
    motor1.drive(-speed);
    motor2.drive(-speed);
    motor3.drive(speed);
    motor4.drive(speed);
}

// Move robot to the right (sideways movement)
inline void moveRight(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4, int speed)
{
    motor1.drive(speed);
    motor2.drive(speed);
    motor3.drive(-speed);
    motor4.drive(-speed);
}

// Move robot forward (straight movement)
inline void moveForward(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4, int speed)
{
    motor1.drive(speed);
    motor2.drive(-speed);
    motor3.drive(-speed);
    motor4.drive(speed);
}

// Move robot backward (straight movement)
inline void moveBackward(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4, int speed)
{
    motor1.drive(-speed);
    motor2.drive(speed);
    motor3.drive(speed);
    motor4.drive(-speed);
}

// #endif // MECANUM_DRIVE_H
