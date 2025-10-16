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
    float alpha = 0.5;  // Smoothing factor (0-1), adjustable for responsiveness
};

// PID parameters (adjustable for tuning)
const float KP_X = 0.4, KI_X = 0.01, KD_X = 0.1;  // For X-axis (left-right)
const float KP_Y = 0.4, KI_Y = 0.01, KD_Y = 0.1;  // For Y-axis (forward-backward)
const float KP_RX = 0.4, KI_RX = 0.01, KD_RX = 0.1;  // For RX-axis (rotation)
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
    if (incomingData.joyData[0] == 0 && incomingData.joyData[1] == 0 && incomingData.joyData[2] == 0 && incomingData.joyData[3] == 0){
        brakeAll(motor1, motor2, motor3, motor4);
    }
    else {
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

        // Apply low-pass filter (EMA)
        float filteredX = applyFilter(x, filterX);
        float filteredY = applyFilter(y, filterY);
        float filteredRX = applyFilter(rx, filterRX);

        // Apply dead zone
        if (abs(filteredX) < DEADZONE) filteredX = 0;
        if (abs(filteredY) < DEADZONE) filteredY = 0;
        if (abs(filteredRX) < DEADZONE) filteredRX = 0;

        // Aplly PID untuk menghaluskan respons
        int smoothedX = applyPIDControl(filteredX, pidX);
        int smoothedY = applyPIDControl(filteredY, pidY);
        int smoothedRX = applyPIDControl(filteredRX, pidRX);

        // Kalkulasi kecepatan motor berdasarkan input joystick yang telah dihaluskan
        int frontLeft  = smoothedY + smoothedX - smoothedRX;
        int backLeft   = smoothedY - smoothedX - smoothedRX;
        int frontRight = smoothedY - smoothedX + smoothedRX;
        int backRight  = smoothedY + smoothedX + smoothedRX;

        // int frontLeft  = smoothedY + smoothedX + smoothedRX;
        // int backLeft   = smoothedY - smoothedX + smoothedRX;
        // int frontRight = smoothedY - smoothedX - smoothedRX;
        // int backRight  = smoothedY + smoothedX - smoothedRX;

        // Normalize motor speeds to stay within -150 to +150 range
        int maxVal = max(max(abs(frontLeft), abs(backLeft)), max(abs(frontRight), abs(backRight)));
        if (maxVal > 100) {
            frontLeft  = (frontLeft  * 100) / maxVal;
            backLeft   = (backLeft   * 100) / maxVal;
            frontRight = (frontRight * 100) / maxVal;
            backRight  = (backRight  * 100) / maxVal;
        }

        // Drive motors with calculated speeds
        motor1.drive(-frontLeft);
        motor2.drive(frontRight);
        motor3.drive(backLeft);
        motor4.drive(-backRight);
    }
}

void GripperControl() {
    unsigned long currentTime = millis();  // Update currentTime each call

    // Lifter control
    if (incomingData.stat[9] == 0 && (currentTime - lastDebounceTimeA) > debounceDelay) { // Tombol A (Lifter down)
        lastDebounceTimeA = currentTime;
        if (lifterState != LIFTER_DOWN) {
        actuationStartTime = currentTime;
        lifterState = LIFTER_MOVING;
        int currentPos = servo2.read();
        moveServoSmooth(servo2, currentPos, 0);
        lifterState = LIFTER_DOWN;
        }
    }

    if (incomingData.stat[11] == 0 && (currentTime - lastDebounceTimeB) > debounceDelay) { // Tombol B (Lifter up)
        lastDebounceTimeB = currentTime;
        if (lifterState != LIFTER_UP) {
        actuationStartTime = currentTime;
        lifterState = LIFTER_MOVING;
        int currentPos = servo2.read();
        moveServoSmooth(servo2, currentPos, 150);
        lifterState = LIFTER_UP;
        }
    }

    // Gripper control
    if (incomingData.stat[13] == 0 && (currentTime - lastDebounceTimeX) > debounceDelay) { // Tombol X (Gripper open)
        lastDebounceTimeX = currentTime;
        if (gripperState != GRIPPER_OPEN) {
            actuationStartTime = currentTime;
            gripperState = GRIPPER_MOVING;
            int currentPos = servo1.read();
            moveServoSmooth(servo1, currentPos, 0);  // Membuka gripper
            gripperState = GRIPPER_OPEN;
        }
    }

    if (incomingData.stat[3] == 0 && (currentTime - lastDebounceTimeY) > debounceDelay) { // Tombol Y (Gripper close)
        lastDebounceTimeY = currentTime;
        if (gripperState != GRIPPER_CLOSE) {
            actuationStartTime = currentTime;
            gripperState = GRIPPER_MOVING;
            int currentPos = servo1.read();
            moveServoSmooth(servo1, currentPos, 180); // Menutup gripper
            gripperState = GRIPPER_CLOSE;
        }
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