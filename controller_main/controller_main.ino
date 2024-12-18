#include <PS4Controller.h>
#include <SPI.h>
#include "RF24.h"

// NRF24L01 setup
RF24 radio(22, 21);  // CE, CSN pins for transmitter
const byte address[] = "00001";

// Data structures
struct __attribute__((packed)) SentData {
    float joystick[2];
    uint8_t state;
};

struct __attribute__((packed)) AckPayload {
    uint8_t state;  // Current robot state
    float currentConsumption;
};

// Enum for robot states
enum RobotState {
    Sit,
    Stand,
    PreWalk,
    Walk
};

RobotState currentState = Sit;  
SentData sentData = {{0.0, 0.0}, Sit};  
AckPayload ackPayload;

unsigned long lastJoystickTime = 0;
unsigned long lastRadioTime = 0;
const unsigned long joystickInterval = 50;
const unsigned long radioInterval = 100;

void initializePS4() {
    PS4.begin();
    Serial.println("Waiting for PS4 controller...");
    while (!PS4.isConnected()) {
        delay(100);
    }
    Serial.println("PS4 controller connected.");
}

void initializeNRF24() {
    if (!radio.begin()) {
        Serial.println("NRF24 initialization failed!");
        while (1);  // Halt the program if NRF24 fails
    }
    radio.setChannel(115);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.enableAckPayload();
    radio.openWritingPipe(address);
    Serial.println("NRF24 initialized successfully.");
}

void processControls() {
    if (PS4.Touchpad()) {
        delay(200);
        if (currentState == Sit) {
            sentData.state = Stand;
            currentState = Stand;
            Serial.println("Mode switched to Stand");
        } else if (currentState == Stand) {
            sentData.state = Sit;
            currentState = Sit;
            Serial.println("Mode switched to Sit");
        } else {
            Serial.println("Cannot switch modes while walking");
        }
    }

    // Map joystick values and handle deadzone
    float rawX = PS4.LStickX();
    float rawY = PS4.LStickY();
    float magnitude = sqrt(rawX * rawX + rawY * rawY);

    if (magnitude > 10) {  // Deadzone threshold
        sentData.joystick[0] = map(rawX, -126, 127, 500, -500);
        sentData.joystick[1] = map(rawY, -126, 127, 500, -500);
    } else {
        sentData.joystick[0] = 0;
        sentData.joystick[1] = 0;
    }

    // Debug output
    Serial.print("Joystick X: ");
    Serial.print(sentData.joystick[0]);
    Serial.print(", Y: ");
    Serial.println(sentData.joystick[1]);
}

void sendData() {
    if (radio.write(&sentData, sizeof(sentData))) {
        if (radio.isAckPayloadAvailable()) {
            radio.read(&ackPayload, sizeof(ackPayload));
            Serial.print("Ack received. Current Robot State: ");
            Serial.println((RobotState)ackPayload.state);
            Serial.print("Ack received. Current Consumption: ");
            Serial.println(ackPayload.currentConsumption);
            if (ackPayload.state >= Sit && ackPayload.state <= Walk) {
                currentState = (RobotState)(ackPayload.state);
            } else {
                Serial.println("Invalid state received in acknowledgment!");
            }
        } else {
            Serial.println("No acknowledgment payload received.");
        }
    } else {
        Serial.println("Failed to send data.");
    }
}

void setup() {
    Serial.begin(115200);
    initializePS4();
    initializeNRF24();
}

void loop() {
    unsigned long currentMillis = millis();

    // Process joystick input periodically
    if (currentMillis - lastJoystickTime >= joystickInterval) {
        lastJoystickTime = currentMillis;
        if (PS4.isConnected()) {
            processControls();
        } else {
            Serial.println("PS4 controller disconnected.");
        }
    }

    // Send data to the robot periodically
    if (currentMillis - lastRadioTime >= radioInterval) {
        lastRadioTime = currentMillis;
        sendData();
    }
}
