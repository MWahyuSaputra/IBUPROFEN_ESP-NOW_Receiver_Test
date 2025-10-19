#ifndef ESPNOW_FUNCTIONS_H
#define ESPNOW_FUNCTIONS_H

#include "Config.h"
#include "Mechanum_Drive.h"
#include "GripperControl.h"

Motor motor1(M1_AIN1, M1_AIN2, M1_PWMA, 1);  // Motor 1 (A)
Motor motor2(M2_BIN1, M2_BIN2, M2_PWMB, 1);  // Motor 2 (B)
Motor motor3(M3_AIN1, M3_AIN, M3_PWMA, 1);  // Motor 3 (A)
Motor motor4(M4_BIN1, M4_BIN2, M4_PWMB, 1);  // Motor 4 (B)

typedef struct struct_message {
    bool stat[15];
    int joyData[4];
    uint8_t remoteIndex = 1;
} struct_message;

struct_message incomingData;

bool isAutoSequenceRunning = false;
bool squarePressed = false;
bool roundPressed = false;
struct PID {
    float Kp, Ki, Kd;
    float prevError;
    float integral;
    float prevOutput;
    PID(float kp, float ki, float kd) : Kp(kp), Ki(ki), Kd(kd), prevError(0), integral(0), prevOutput(0) {}
};

// Low-pass filter struct using Exponential Moving Average (EMA)
struct AxisFilter {
    float prevFiltered = 0;
    float alpha = 0.55;  // Smoothing factor (0-1), adjustable for responsiveness
    // Note: alpha for RX will be overridden to 0.8 for smoother rotation
};

// PID parameters (adjustable for tuning)
const float KP_X = 0.25, KI_X = 0.01, KD_X = 0.15;  // For X-axis (left-right)
const float KP_Y = 0.25, KI_Y = 0.01, KD_Y = 0.15;  // For Y-axis (forward-backward)
const float KP_RX = 0.25, KI_RX = 0.01, KD_RX = 0.15;  // For RX-axis (rotation) - reduced for smoother response
const int DEADZONE = 5;  // Dead zone threshold to prevent vibrations at neutral

// Function to apply low-pass filter (EMA) to axis input
float applyFilter(int input, AxisFilter &filter) {
    filter.prevFiltered = filter.alpha * input + (1 - filter.alpha) * filter.prevFiltered;
    return filter.prevFiltered;
}

// Function to apply PID control for smoothing
int applyPIDControl(float setpoint, PID &pid) {
    float error = setpoint - pid.prevOutput;
    pid.integral += error;
    // Limit integral to prevent windup
    if (pid.integral > 100) pid.integral = 100;
    if (pid.integral < -100) pid.integral = -100;
    float derivative = error - pid.prevError;
    float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    pid.prevError = error;
    pid.prevOutput = output;
    return (int)output;
}

void DriveRobot(){
    // Check if joystick is active (any axis not zero)
    bool joystickActive = false;
    for(int i = 0; i < 4; i++){
        if(incomingData.joyData[i] != 0){
            joystickActive = true;
            break;
        }
    }

    if(joystickActive){
        // Joystick mode: use PID and low-pass filter for smooth movement
        // persistent filter and PID instances for each axis
        static AxisFilter filterX, filterY, filterRX;
        static PID pidX = {KP_X, KI_X, KD_X};
        static PID pidY = {KP_Y, KI_Y, KD_Y};
        static PID pidRX = {KP_RX, KI_RX, KD_RX};

        // Extract raw joystick data
        int x = incomingData.joyData[0];  // X-axis (Left-Right movement)
        int y = incomingData.joyData[1];  // Y-axis (Forward-Backward movement)
        int rx = incomingData.joyData[2]; // X-axis (Rotation)
        int ry = incomingData.joyData[3]; // Y-axis (Unused in this case)

        // Set higher smoothing for rotation
        filterRX.alpha = 0.8;

        // Apply low-pass filter (EMA)
        float filteredX = applyFilter(x, filterX);
        float filteredY = applyFilter(y, filterY);
        float filteredRX = applyFilter(rx, filterRX);

        // Apply dead zone
        if (abs(filteredX) < DEADZONE) filteredX = 0;
        if (abs(filteredY) < DEADZONE) filteredY = 0;
        if (abs(filteredRX) < DEADZONE) filteredRX = 0;

        // Apply PID untuk menghaluskan respons
        int smoothedX = applyPIDControl(filteredX, pidX);
        int smoothedY = applyPIDControl(filteredY, pidY);
        int smoothedRX = applyPIDControl(filteredRX, pidRX);

        // Kalkulasi kecepatan motor berdasarkan input joystick yang telah dihaluskan
        // Scale rotation contribution to limit speed
        int scaledRX = smoothedRX * 0.075; // Scale down rotation effect to 60%
        // int frontLeft  = smoothedY + smoothedX + scaledRX;
        // int backLeft   = smoothedY - smoothedX + scaledRX;
        // int frontRight = smoothedY - smoothedX - scaledRX;
        // int backRight  = smoothedY + smoothedX - scaledRX;
        int frontLeft  = smoothedY - smoothedX + scaledRX;
        int backLeft   = smoothedY + smoothedX + scaledRX;
        int frontRight = smoothedY + smoothedX - scaledRX;
        int backRight  = smoothedY - smoothedX - scaledRX;

        // Normalize motor speeds
        int maxVal = max(max(abs(frontLeft), abs(backLeft)), max(abs(frontRight), abs(backRight)));
        if (maxVal > 100) {
            frontLeft  = (frontLeft  * 100) / maxVal;
            backLeft   = (backLeft   * 100) / maxVal;
            frontRight = (frontRight * 100) / maxVal;
            backRight  = (backRight  * 100) / maxVal;
        }

        // Drive motors with calculated speeds
        motor1.drive(-frontLeft * 0.975);
        motor2.drive(frontRight * 0.85);
        motor3.drive(backLeft   * 0.975);
        motor4.drive(-backRight * 0.85);
    } else {
        // Button mode: constant speed movement using same calculation as joystick for consistency
        const int buttonSpeed = 75;  // Constant speed for button controls
        int x = 0, y = 0, rx = 0;
        if(incomingData.stat[4] == 0){  // UP button pressed
            y = -buttonSpeed;
        } else if(incomingData.stat[6] == 0){  // DOWN button pressed
            y = buttonSpeed;
        } else if(incomingData.stat[5] == 0){  // LEFT button pressed
            x = -buttonSpeed;
        } else if(incomingData.stat[7] == 0){  // RIGHT button pressed
            x = buttonSpeed;
        } else {
            // No buttons pressed, brake all motors
            brakeAll(motor1, motor2, motor3, motor4);
            return;
        }

        // Use same motor speed calculation as joystick mode
        int scaledRX = rx * 0.075; // Scale down rotation effect
        int frontLeft  = y - x + scaledRX;
        int backLeft   = y + x + scaledRX;
        int frontRight = y + x - scaledRX;
        int backRight  = y - x - scaledRX;

        // Normalize motor speeds
        int maxVal = max(max(abs(frontLeft), abs(backLeft)), max(abs(frontRight), abs(backRight)));
        if (maxVal > 100) {
            frontLeft  = (frontLeft  * 100) / maxVal;
            backLeft   = (backLeft   * 100) / maxVal;
            frontRight = (frontRight * 100) / maxVal;
            backRight  = (backRight  * 100) / maxVal;
        }

        // Drive motors with calculated speeds
        motor1.drive(-frontLeft * 0.975);
        motor2.drive(frontRight * 0.85);
        motor3.drive(backLeft   * 0.975);
        motor4.drive(-backRight * 0.85);
    }
}

void TakeObject() {
    // Move forward briefly to approach the object
    moveForward(motor1, motor2, motor3, motor4, 30);
    delay(175);
    brakeAll(motor1, motor2, motor3, motor4);
    delay(100);

    // Move backward a little to align object's position with gripper
    moveBackward(motor1, motor2, motor3, motor4, 30);
    delay(245);
    brakeAll(motor1, motor2, motor3, motor4);

    lifterState = LIFTER_MOVING;
    actuationStartTime = millis();
    // Move lifter down
    int currentLifterPos = servo2.read();
    moveServoSmooth(servo2, currentLifterPos, 0);
    lifterState = LIFTER_DOWN;
    // Close gripper
    gripperState = GRIPPER_MOVING;
    int currentGripperPos = servo1.read();
    moveServoSmooth(servo1, currentGripperPos, 180);
    gripperState = GRIPPER_CLOSE;
    // Move lifter up
    lifterState = LIFTER_MOVING;
    moveServoSmooth(servo2, 0, 150);
    lifterState = LIFTER_UP;
    lifterPosition = 150;  // Update position after auto sequence
}

void PlaceObject() {
    // Move backward a little before lowering lifter
    moveBackward(motor1, motor2, motor3, motor4, 30);
    delay(275);
    brakeAll(motor1, motor2, motor3, motor4);

    lifterState = LIFTER_MOVING;
    actuationStartTime = millis();
    // Move lifter down
    int currentLifterPos = servo2.read();
    moveServoSmooth(servo2, currentLifterPos, 45);
    lifterState = LIFTER_DOWN;
    // Open gripper
    gripperState = GRIPPER_MOVING;
    int currentGripperPos = servo1.read();
    moveServoSmooth(servo1, currentGripperPos, 0);
    gripperState = GRIPPER_OPEN;
    // Move lifter up
    lifterState = LIFTER_MOVING;
    moveServoSmooth(servo2, 0, 150);
    lifterState = LIFTER_UP;
}

void GripperControl() {
    unsigned long currentTime = millis();  // Update currentTime each call
// Lifter control - incremental
if (incomingData.stat[9] == 0 && (currentTime - lastDebounceTimeA) > debounceDelay && !isAutoSequenceRunning) { // Tombol X (Lifter down)
    lastDebounceTimeA = currentTime;
    lifterPosition = constrain(lifterPosition - 25, 0, 150);
    actuationStartTime = currentTime;
    lifterState = LIFTER_MOVING;
    int currentPos = servo2.read();
    moveServoSmooth(servo2, currentPos, lifterPosition);
    lifterState = (lifterPosition == 0) ? LIFTER_DOWN : (lifterPosition == 150) ? LIFTER_UP : LIFTER_MOVING;
}
if (incomingData.stat[11] == 0 && (currentTime - lastDebounceTimeB) > debounceDelay && !isAutoSequenceRunning) { // Tombol Triangle (Lifter up)
    lastDebounceTimeB = currentTime;
    lifterPosition = constrain(lifterPosition + 25, 0, 150);
    actuationStartTime = currentTime;
    lifterState = LIFTER_MOVING;
    int currentPos = servo2.read();
    moveServoSmooth(servo2, currentPos, lifterPosition);
    lifterState = (lifterPosition == 0) ? LIFTER_DOWN : (lifterPosition == 150) ? LIFTER_UP : LIFTER_MOVING;
}
// Lifter control - full movement
if (incomingData.stat[2] == 0 && (currentTime - lastDebounceTimeL2) > debounceDelay && !isAutoSequenceRunning) { // Tombol L2 (Lifter down full)
    lastDebounceTimeL2 = currentTime;
    actuationStartTime = currentTime;
    lifterState = LIFTER_MOVING;
    int currentPos = servo2.read();
    moveServoSmooth(servo2, currentPos, 0);
    lifterState = LIFTER_DOWN;
    lifterPosition = 0;
}
if (incomingData.stat[12] == 0 && (currentTime - lastDebounceTimeR2) > debounceDelay && !isAutoSequenceRunning) { // Tombol R2 (Lifter up full)
    lastDebounceTimeR2 = currentTime;
    actuationStartTime = currentTime;
    lifterState = LIFTER_MOVING;
    int currentPos = servo2.read();
    moveServoSmooth(servo2, currentPos, 150);
    lifterState = LIFTER_UP;
    lifterPosition = 150;
}
// Gripper control
if (incomingData.stat[13] == 0 && (currentTime - lastDebounceTimeX) > debounceDelay && !isAutoSequenceRunning) { // Tombol L1 (Gripper open)
    lastDebounceTimeX = currentTime;
    if (gripperState != GRIPPER_OPEN) {
        actuationStartTime = currentTime;
        gripperState = GRIPPER_MOVING;
        int currentPos = servo1.read();
        moveServoSmooth(servo1, currentPos, 0);  // Membuka gripper
        gripperState = GRIPPER_OPEN;
    }
}
if (incomingData.stat[3] == 0 && (currentTime - lastDebounceTimeY) > debounceDelay && !isAutoSequenceRunning) { // Tombol R1 (Gripper close)
    lastDebounceTimeY = currentTime;
    if (gripperState != GRIPPER_CLOSE) {
        actuationStartTime = currentTime;
        gripperState = GRIPPER_MOVING;
        int currentPos = servo1.read();
        moveServoSmooth(servo1, currentPos, 180); // Menutup gripper
        gripperState = GRIPPER_CLOSE;
    }
}
    // Automatic Take Object (Square button)
    if (incomingData.stat[8] == 0 && !squarePressed && !isAutoSequenceRunning) {
        squarePressed = true;
        isAutoSequenceRunning = true;
        TakeObject();
        isAutoSequenceRunning = false;
    }
    if (incomingData.stat[8] == 1 && squarePressed) {
        squarePressed = false;
    }
    // Automatic Place Object (Round button)
    if (incomingData.stat[10] == 0 && !roundPressed && !isAutoSequenceRunning) {
        roundPressed = true;
        isAutoSequenceRunning = true;
        PlaceObject();
        isAutoSequenceRunning = false;
    }
    if (incomingData.stat[10] == 1 && roundPressed) {
        roundPressed = false;
    }
    // Timed actuation check (stop if max time exceeded, though soft-move handles timing)
    if (lifterState == LIFTER_MOVING && (currentTime - actuationStartTime) > maxActuationTime) {
        servo2.write(servo2.read());
        lifterState = (servo2.read() < 75) ? LIFTER_DOWN : LIFTER_UP;
    }
    if (gripperState == GRIPPER_MOVING && (currentTime - actuationStartTime) > maxActuationTime) {
        servo1.write(servo1.read());
        gripperState = (servo1.read() < 90) ? GRIPPER_OPEN : GRIPPER_CLOSE;
    }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingLocal, int len) {
    memcpy(&incomingData, incomingLocal, sizeof(incomingData));
    String dataLine = String();
    for (int i = 0; i < 4; i++) {  dataLine += String(incomingData.joyData[i]) + ","; }
    for (int i = 0; i < 15; i++) { dataLine += String(incomingData.stat[i]) + ",";    }
    
    dataLine += String(incomingData.remoteIndex);
    DEBUG_PRINTLN("Serial Sent Data: " + dataLine);
    
    DriveRobot();
    GripperControl();
    // DriveMotorButtonlogic();
    // ShortCutSpeedControl();
    // failSafeCheck(incomingData);
}

void startComms() {
    if (esp_now_init() != ESP_OK) {
        DEBUG_PRINTLN("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
}

void failSafeCheck(struct_message &recvData) {
    static struct_message lastReceivedData;
    static unsigned long lastReceiveTime = 0;
    static bool failsafeTriggered = false;

    bool dataChanged = false;
    for (int i = 0; i < 4; i++) {
        if (recvData.joyData[i] != lastReceivedData.joyData[i]) {
            dataChanged = true;
            break;
        }
    }
    if (!dataChanged) {
        for (int i = 0; i < 15; i++) {
            if (recvData.stat[i] != lastReceivedData.stat[i]) {
                dataChanged = true;
                break;
            }
        }
    }
    if (dataChanged) {
        lastReceiveTime = millis();
        lastReceivedData = recvData;
        failsafeTriggered = false;
    }
    if (!failsafeTriggered && (millis() - lastReceiveTime >= 1000)) {
        bool joystickMoved = false;
        bool buttonPressed = false;
        for (int i = 0; i < 4; i++) {
            if (lastReceivedData.joyData[i] != 0) {
                joystickMoved = true;
                break;
            }
        }
        for (int i = 0; i < 15; i++) {
            if (!lastReceivedData.stat[i]) {
                buttonPressed = true;
                break;
            }
        }
        if (joystickMoved || buttonPressed) {
            for (int i = 0; i < 4; i++) recvData.joyData[i] = 0;
            for (int i = 0; i < 15; i++) recvData.stat[i] = true;
            String dataLine;

            for (int i = 0; i < 4; i++)  dataLine += String(recvData.joyData[i]) + ",";
            for (int i = 0; i < 15; i++) dataLine += String(recvData.stat[i]) + ",";
            dataLine += String(recvData.remoteIndex);
            Serial2.println(dataLine);
            DEBUG_PRINTLN("FAILSAFE: RESETTING THE SERIAL DATA");
            DEBUG_PRINTLN("Serial Sent Data: " + dataLine);

            failsafeTriggered = true;
        }
    }
}

#endif // ESPNOW_FUNCTIONS_H