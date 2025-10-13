#include "Config.h"
#include "Mechanum_Drive.h"

Motor motor1(M1_AIN1, M1_AIN2, M1_PWMA, 1);  // Motor 1 (A)
Motor motor2(M2_BIN1, M2_BIN2, M2_PWMB, 1);  // Motor 2 (B)
Motor motor3(M3_AIN1, M3_AIN, M3_PWMA, 1);  // Motor 3 (A)
Motor motor4(M4_BIN1, M4_BIN2, M4_PWMB, 1);  // Motor 4 (B)

typedef struct struct_message {
    bool stat[15];
    int joyData[4];
    uint8_t remoteIndex = 1;
//    char address[18];
} struct_message;

struct_message incomingData; 

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingLocal, int len) {
    memcpy(&incomingData, incomingLocal, sizeof(incomingData));
    String dataLine = String();
    for (int i = 0; i < 4; i++) {  dataLine += String(incomingData.joyData[i]) + ","; }
    for (int i = 0; i < 15; i++) { dataLine += String(incomingData.stat[i]) + ",";    }
    
    dataLine += String(incomingData.remoteIndex);

    DEBUG_PRINTLN("Serial Sent Data: " + dataLine);

    if (incomingData.joyData[0] == 0 && incomingData.joyData[1] == 0 && incomingData.joyData[2] == 0 && incomingData.joyData[3] == 0){
        brakeAll(motor1, motor2, motor3, motor4);
    }
    else {
        int x = incomingData.joyData[0];  // X-axis (Left-Right movement)
        int y = incomingData.joyData[1];  // Y-axis (Forward-Backward movement)
        int rx = incomingData.joyData[2]; // X-axis (Rotation)
        int ry = incomingData.joyData[3]; // Y-axis (Unused in this case)

        int frontLeft  = y + x + rx;  
        int backLeft   = y - x + rx;  
        int frontRight = y - x - rx; 
        int backRight  = y + x - rx;  

        int maxVal = max(max(abs(frontLeft), abs(backLeft)), max(abs(frontRight), abs(backRight)));
        if (maxVal > 150) {
            frontLeft  = (frontLeft  * 150) / maxVal;
            backLeft   = (backLeft   * 150) / maxVal;
            frontRight = (frontRight * 150) / maxVal;
            backRight  = (backRight  * 150) / maxVal;
        }

        motor1.drive(-frontLeft);
        motor2.drive(frontRight);
        motor3.drive(backLeft);
        motor4.drive(-backRight);
    }
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