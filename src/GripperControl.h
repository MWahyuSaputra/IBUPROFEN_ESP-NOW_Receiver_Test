#include <ESP32Servo.h>

unsigned long currentTime = millis();

Servo servo1;  // servo pertama (gripper)
Servo servo2;  // servo kedua (lifter)

int servoPin1 = 12;  // pin servo 1 (gripper)
int servoPin2 = 14;  // pin servo 2 (lifter)

// State management enums
enum LifterState { LIFTER_DOWN, LIFTER_UP, LIFTER_MOVING };
enum GripperState { GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_MOVING };

LifterState lifterState = LIFTER_DOWN;  // Assume starting at down
GripperState gripperState = GRIPPER_OPEN; // Assume starting at open

// Debounce and timing variables
unsigned long lastDebounceTimeA = 0;
unsigned long lastDebounceTimeB = 0;
unsigned long lastDebounceTimeX = 0;
unsigned long lastDebounceTimeY = 0;  // Assuming stat[3] is Y button
const unsigned long debounceDelay = 200;  // 200ms debounce

// Timed actuation variables
unsigned long actuationStartTime = 0;
const unsigned long maxActuationTime = 2000;  // Max 2 seconds per actuation

// Soft-start/soft-stop parameters
const int stepDelay = 20;  // Delay between steps in ms
const int stepSize = 5;    // Degrees per step

void setupGripperControl() {
  // Timer untuk kontrol PWM servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo1.attach(servoPin1, 1000, 2000);
    servo2.attach(servoPin2, 1000, 2000);

    servo1.write(0);  // Gripper open
    servo2.write(0);  // Lifter down
}

void moveServoSmooth(Servo &servo, int startPos, int endPos) {
    int currentPos = startPos;
    int direction = (endPos > startPos) ? 1 : -1;
    while (currentPos != endPos) {
        currentPos += direction * stepSize;
        if ((direction == 1 && currentPos > endPos) || (direction == -1 && currentPos < endPos)) {
        currentPos = endPos;
        }
        servo.write(currentPos);
        delay(stepDelay);
    }
    }