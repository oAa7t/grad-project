#include <SPI.h>
#include "RF24.h"
#define ACS712_PIN A0 // Analog pin connected to ACS712 output
#define ACS712_SENSITIVITY 0.066 // Sensitivity in V/A for ACS712-30A
#define ACS712_OFFSET 2.5 // Voltage at zero current (center point)

// NRF24L01 instance
RF24 radio(9, 10);  // CE, CSN pins for receiver

// Define pipe address
const byte address[] = "00001";

// Struct for received data
struct __attribute__((packed)) ReceivedData {
    float joystick[2];
    uint8_t state;
};

// Struct for acknowledgment payload
struct __attribute__((packed)) AckPayload {
    uint8_t state;      // Current robot state
    float currentConsumption;
};

// Enum for robot states
enum RobotState {
    Sit,
    Stand,
    PreWalk,
    Walk
};

ReceivedData receivedData;
AckPayload ackPayload;

float measureCurrent() {
  int adcValue = analogRead(ACS712_PIN); // Read raw ADC value (0-1023)
  float voltage = adcValue * (5.0 / 1023.0); // Convert ADC value to voltage
  float current = (voltage - ACS712_OFFSET) / ACS712_SENSITIVITY; // Calculate current
  return current; // Return the measured current in amps
}

// Function to initialize communication
void initializeCommunication() {
    if (!radio.begin()) {
        Serial.println("NRF24 initialization failed!");
        while (1); // Halt the program if initialization fails
    }

    // NRF24L01 settings
    radio.setChannel(115);               // Ensure no conflict with WiFi (2.4 GHz range)
    radio.setPALevel(RF24_PA_HIGH);      // Set power level to high
    radio.setDataRate(RF24_250KBPS);     // Use lower data rate for longer range
    radio.openReadingPipe(1, address);  // Open pipe for reading data
    radio.enableAckPayload();           // Enable acknowledgment payloads
    radio.startListening();             // Start listening mode

    Serial.println("NRF24 initialized successfully.");
}

// Function to process incoming data and send acknowledgment
void processData(ReceivedData& receivedData, AckPayload ackPayload) {
    if (radio.available()) {
        // Receive the data
        radio.read(&receivedData, sizeof(receivedData));

        // Debugging output
        Serial.print("Joystick X: ");
        Serial.print(receivedData.joystick[0]);
        Serial.print(", Y: ");
        Serial.print(receivedData.joystick[1]);
        Serial.print(", State: ");
        Serial.println(receivedData.state);

        radio.writeAckPayload(1, &ackPayload, sizeof(ackPayload)); // This will be prepared in the main file since states changes based on robot movements
    } //else Serial.println("Connection Error!");
}
