#include "hex_move.h"       // Include movement functions

RobotState currentState = Stand; // Default state is Sit
unsigned long lastReceivedTime = 0; // For connection timeout handling
unsigned long previousWalkTime = 0;
unsigned long currentTime;

// Timing intervals (in milliseconds)
const unsigned long walkInterval = 50;   // Time interval for walking updates

void setup() {
    Serial.begin(115200);
    initializePCA();         // Initialize servo controllers
    initializeStepInterval(); // Initialize speed calculations
    initializeCommunication(); // Initialize NRF communication
    //sit();
    Serial.println("Initializing in Stand...");
    stand(); // Start in stand mode
    currentState = Stand;
}

void loop() {
    // Prepare received and acknowledgment data structure
    currentTime = millis();

    // Process incoming data
    processData(receivedData, ackPayload);

    // Calculate joystick magnitude
    float magnitude = sqrt(pow(receivedData.joystick[0], 2) + pow(receivedData.joystick[1], 2));

    Serial.print("Joystick Magnitude: ");
    Serial.println(magnitude);

    // State transitions based on joystick magnitude
    if (magnitude < 100 && currentState == Stand) {
            stand(); // Call the stand function
            
    } if (magnitude >= 100) {
            currentState = Walk;
            preWalk(); // Call the preWalk function
    } if (magnitude < 100 && currentState != Stand) {
      currentState = Stand;
    }

    // Update acknowledgment payload
    ackPayload.state = currentState;
    ackPayload.currentConsumption = measureCurrent();
}
