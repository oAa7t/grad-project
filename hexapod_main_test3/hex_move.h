#ifndef HEX_MOVE_H
#define HEX_MOVE_H

#include <Adafruit_PWMServoDriver.h>
#include "communication.h"


// Define constants
const float Robot_height = 100.0;
const float path_radius = 50.0;
const float motor_distance = 150.0;
const float hex_radius = motor_distance + 200.0;

// Angles in radians
const float angle_L1 = M_PI / 6.0;  // 30 degrees
const float angle_L2 = 0.0;         // 0 degrees
const float angle_L3 = -M_PI / 6.0; // -30 degrees
const float angle_R1 = 5 * M_PI / 6.0;  // 150 degrees
const float angle_R2 = M_PI;            // 180 degrees
const float angle_R3 = -5 * M_PI / 6.0; // -150 degrees

// Path centers for legs
const float path_center_L1[] = {path_radius * cos(angle_L1), 165.0, Robot_height};
const float path_center_L2[] = {path_radius * cos(angle_L2), 165.0, Robot_height};
const float path_center_L3[] = {path_radius * cos(angle_L3), 165.0, Robot_height};
const float path_center_R1[] = {path_radius * cos(angle_R1), 165.0, Robot_height};
const float path_center_R2[] = {path_radius * cos(angle_R2), 165.0, Robot_height};
const float path_center_R3[] = {path_radius * cos(angle_R3), 165.0, Robot_height};

unsigned long previousTimeLeft[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // For Left1, Left2, Left3
unsigned long previousTimeRight[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // For Right1, Right2, Right3

float directionVector[2] = {0.0f, 0.0f};

bool standing;
/*
const float R1o[] = {0.0, 0.0, 0.0};
const float R2o[] = {0.0, 0.0, 0.0};
const float R3o[] = {0.0, 0.0, 0.0};
const float L1o[] = {0.0, 0.0, 0.0};
const float L2o[] = {0.0, 0.0, 0.0};
const float L3o[] = {0.0, 0.0, 0.0};
*/
const float R1o[] = {10.0, 0.0, -15.0};
const float R2o[] = {15.0, 0.0, -15.0};
const float R3o[] = {20.0, 5.0, -15.0};
const float L1o[] = {-20.0, 0.0, 15.0};
const float L2o[] = {-8.0, 0.0, 15.0};
const float L3o[] = {-20.0, 0.0, 15.0};

// Create legs (servo indices for each leg)
const int Left1[] = {0, 1, 2};
const int Left2[] = {3, 4, 5};
const int Left3[] = {6, 7, 8};
const int Right1[] = {0, 1, 2};
const int Right2[] = {3, 4, 5};
const int Right3[] = {6, 7, 8};

float step_interval = 0.0;

// Initialize PCA9685 servo driver instances
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40); // Address for PCA1
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41); // Address for PCA2

// Define servo range (pulse widths in microseconds)
const int SERVO_MIN = 70; // Minimum pulse length
const int SERVO_MAX = 520; // Maximum pulse length

// Initialize PCA boards
void initializePCA() {
    pca1.begin();
    pca1.setPWMFreq(50);
    pca2.begin();
    pca2.setPWMFreq(50);
}

// Function to map angle to pulse width
int mapAngleToPulse(int angle) {
    return map(angle, 180, 0, SERVO_MAX, SERVO_MIN);
}

// Function to set servo angle
void setServoAngle(Adafruit_PWMServoDriver& pca, int channel, int angle) {
    int pulseLength = mapAngleToPulse(90 - angle);
    Serial.print("Angle: ");
    Serial.print(90 - angle);
    Serial.print(" Pulse: ");
    Serial.println(pulseLength);
    pca.setPWM(channel, 0, pulseLength);
}

// Function to emulate linspace
void linspaceEmulation(float start, float stop, int num, float* result) {
    float step = (stop - start) / (num - 1);
    for (int i = 0; i < num; ++i) {
        result[i] = start + i * step;
    }
}

// Function to calculate inverse kinematics
void calculateAngles(float x_p, float y_p, float z_p, float* angles) {
    const float L1 = 47.0;
    const float L2 = 88.46;
    const float L3 = 148.8;

    float A_p = sqrt(x_p * x_p + y_p * y_p);
    float B_p = sqrt(pow(A_p - L1, 2) + z_p * z_p);

    float theta_1 = atan2(y_p, x_p) * 180 / M_PI;
    float beta = acos((L2 * L2 + B_p * B_p - L3 * L3) / (2 * L2 * B_p)) * 180 / M_PI;
    float epsilon = atan2(A_p - L1, z_p) * 180 / M_PI;

    float sigma_2 = beta - (90 - epsilon);
    float sigma_3 = acos((L2 * L2 + L3 * L3 - B_p * B_p) / (2 * L2 * L3)) * 180 / M_PI;

    angles[0] = 90 - theta_1;
    angles[1] = sigma_2;
    angles[2] = 180 - sigma_3;
}


// Function to set a leg to standby position
void standbyPosition(float x_p, float y_p, float z_p, const int* leg, const char* order, const float* O) {
    float angles[3];
    calculateAngles(x_p, y_p, z_p, angles);

    if (strcmp(order, "left") == 0) {
          setServoAngle(pca1, leg[0], angles[0] + O[0]);
          delay(1);
          setServoAngle(pca1, leg[1], angles[1] + O[1]);
          delay(1);
          setServoAngle(pca1, leg[2], angles[2] - 90 + O[2]);
          delay(1);
    } else if (strcmp(order, "right") == 0) {
          setServoAngle(pca2, leg[0], angles[0] + O[0]);
          delay(1);
          setServoAngle(pca2, leg[1], -angles[1] + O[1]);
          delay(1);
          setServoAngle(pca2, leg[2], -angles[2] + 90 + O[2]);
          delay(1);
    }
}

// Function to calculate path resolution and step interval from speed
void getResFromSpeed(float V_hex, float* Res) {
    const float V_m_angular = 315.0; // degrees/s
    const float V_m = (V_m_angular / 360.0) * (2 * M_PI * 200.0); // mm/s
    const float C_unit = 300.0; // units per second
    const float U_step = 2.0 * 50.0;

    // Calculate U_d
    float U_d = V_m / (C_unit * (1.0 + V_m * ((1.0 / V_hex) - (1.0 / V_m))));

    // Calculate step interval
    float step_interval1 = U_d * ((1.0 / V_hex) - (1.0 / V_m));

    // Calculate resolution
    float path_resolution = U_step / (((1.0 / C_unit) - step_interval1) * V_m);
    Res[0] = path_resolution; 
    Res[1] = step_interval1;
}

void initializeStepInterval() {
  float StepPath[2];
  getResFromSpeed(1100.0, StepPath);
  step_interval = StepPath[1];
}

void preGait(const int* leg, const float* O, const float* path_center, float path_radius, const float* direction_vector, float leg_angle, float hex_radius, const char* order, const char* side, float i2, float j2, float k2) {
    unsigned long currentTime = millis();
    unsigned long previousTime = 0;
    float x_L, y_L, z_L;

    if (strcmp(order, "drag") == 0) {
        // Drag Forward
        float x_L_F = (0.0f - j2) * cos(leg_angle);
        float y_L_F = (0.0f - j2) * sin(leg_angle);
        float z_L_F = 0.0f;

        // Turn
        float x_L_T = -(0.0f - 249.0f * k2 * tan(M_PI / 18.0));
        float y_L_T = 0.0f;

        // Calculate final positions
        x_L = path_center[0] + direction_vector[1] * x_L_F + direction_vector[0] * x_L_T;
        y_L = path_center[1] + direction_vector[1] * y_L_F + fabs(direction_vector[0] * y_L_T);
        z_L = path_center[2];

        float L_angles[3];
        calculateAngles(x_L, y_L, z_L, L_angles);

        if (strcmp(side, "left") == 0) {

            if (millis() - previousTimeLeft[leg[0] / 3][0] >= step_interval) {
                setServoAngle(pca1, leg[0], L_angles[0] + O[0]);
                previousTimeLeft[leg[0] / 3][0] = millis();
            }
            if (millis() - previousTimeLeft[leg[0] / 3][1] >= step_interval) {
                setServoAngle(pca1, leg[1], L_angles[1] + O[1]);
                previousTimeLeft[leg[0] / 3][1] = millis();
            }
            if (millis() - previousTimeLeft[leg[0] / 3][2] >= step_interval) {
                setServoAngle(pca1, leg[2], L_angles[2] + O[2] - 90);
                previousTimeLeft[leg[0] / 3][2] = millis();
            }

        } else if (strcmp(side, "right") == 0) {

            if (millis() - previousTimeRight[leg[0] / 3][0] >= step_interval) {
                setServoAngle(pca2, leg[0], L_angles[0] + O[0]);
                previousTimeRight[leg[0] / 3][0] = millis();
            }
            if (millis() - previousTimeRight[leg[0] / 3][1] >= step_interval) {
                setServoAngle(pca2, leg[1], -L_angles[1] + O[1]);
                previousTimeRight[leg[0] / 3][1] = millis();
            }
            if (millis() - previousTimeRight[leg[0] / 3][2] >= step_interval) {
                setServoAngle(pca2, leg[2], -L_angles[2] + 90 + O[2]);
                previousTimeRight[leg[0] / 3][2] = millis();
            }
        }

    } else if (strcmp(order, "lift") == 0) {
        // Lift Forward
        float x_L_lift_F = (0.0f + j2) * cos(leg_angle);
        float y_L_lift_F = (0.0f + j2) * sin(leg_angle);
        float z_L_lift_F = -path_radius * sin(i2);

        // Turn
        float x_L_T = -(0.0f + 249.0f * k2 * tan(M_PI / 18.0));
        float y_L_T = 0.0f;

        // Calculate final positions
        x_L = path_center[0] + direction_vector[1] * x_L_lift_F + direction_vector[0] * x_L_T;
        y_L = path_center[1] + direction_vector[1] * y_L_lift_F + fabs(direction_vector[0] * y_L_T);
        z_L = path_center[2] - path_radius * sin(i2);

        float L_angles[3];
        calculateAngles(x_L, y_L, z_L, L_angles);

        if (strcmp(side, "left") == 0) {

            if (millis() - previousTimeLeft[leg[0] / 3][0] >= step_interval) {
                setServoAngle(pca1, leg[0], L_angles[0] + O[0]);
                previousTimeLeft[leg[0] / 3][0] = millis();
            }
            if (millis() - previousTimeLeft[leg[0] / 3][1] >= step_interval) {
                setServoAngle(pca1, leg[1], L_angles[1] + O[1]);
                previousTimeLeft[leg[0] / 3][1] = millis();
            }
            if (millis() - previousTimeLeft[leg[0] / 3][2] >= step_interval) {
                setServoAngle(pca1, leg[2], L_angles[2] + O[2] - 90);
                previousTimeLeft[leg[0] / 3][2] = millis();
            }

        } else if (strcmp(side, "right") == 0) {

            if (millis() - previousTimeRight[leg[0] / 3][0] >= step_interval) {
                setServoAngle(pca2, leg[0], L_angles[0] + O[0]);
                previousTimeRight[leg[0] / 3][0] = millis();
            }
            if (millis() - previousTimeRight[leg[0] / 3][1] >= step_interval) {
                setServoAngle(pca2, leg[1], -L_angles[1] + O[1]);
                previousTimeRight[leg[0] / 3][1] = millis();
            }
            if (millis() - previousTimeRight[leg[0] / 3][2] >= step_interval) {
                setServoAngle(pca2, leg[2], -L_angles[2] + 90 + O[2]);
                previousTimeRight[leg[0] / 3][2] = millis();
            }
        }
    }
}



void gait(const int* leg, const float* O, const float* path_center, float path_radius, const float* direction_vector, float leg_angle, float hex_radius, const char* order, const char* side, float i, float j, float k) {
    unsigned long currentTime = millis();
    unsigned long previousTime = 0;
    float x_L, y_L, z_L;

    if (strcmp(order, "drag") == 0) {
        // Drag Forward
        float x_L_F = (path_radius - j) * cos(leg_angle);
        float y_L_F = (path_radius - j) * sin(leg_angle);
        float z_L_F = 0.0f;

        float x_L_T = -(249.0f * tan(M_PI / 18.0) - 2.0f * 249.0f * k * tan(M_PI / 18.0));
        float y_L_T = 0.0f;

        // Calculate final positions
        x_L = path_center[0] + direction_vector[1] * x_L_F + direction_vector[0] * x_L_T;
        y_L = path_center[1] + direction_vector[1] * y_L_F + fabs(direction_vector[0] * y_L_T);
        z_L = path_center[2];

        // Calculate angles and set servos
        float L_angles[3];
        calculateAngles(x_L, y_L, z_L, L_angles);

        if (strcmp(side, "left") == 0) {

            if (millis() - previousTimeLeft[leg[0] / 3][0] >= step_interval) {
                setServoAngle(pca1, leg[0], L_angles[0] + O[0]);
                previousTimeLeft[leg[0] / 3][0] = millis();
            }

            if (millis() - previousTimeLeft[leg[0] / 3][1] >= step_interval) {
                setServoAngle(pca1, leg[1], L_angles[1] + O[1]);
                previousTimeLeft[leg[0] / 3][1] = millis();
            }

            if (millis() - previousTimeLeft[leg[0] / 3][2] >= step_interval) {
                setServoAngle(pca1, leg[2], L_angles[2] + O[2] - 90);
                previousTimeLeft[leg[0] / 3][2] = millis();
            }
            
        } else if (strcmp(side, "right") == 0) {

            if (millis() - previousTimeRight[leg[0] / 3][0] >= step_interval) {
                setServoAngle(pca2, leg[0], L_angles[0] + O[0]);
                previousTimeRight[leg[0] / 3][0] = millis();
            }

            if (millis() - previousTimeRight[leg[0] / 3][1] >= step_interval) {
                setServoAngle(pca2, leg[1], -L_angles[1] + O[1]);
                previousTimeRight[leg[0] / 3][1] = millis();
            }

            if (millis() - previousTimeRight[leg[0] / 3][2] >= step_interval) {
                setServoAngle(pca2, leg[2], -L_angles[2] + 90 + O[2]);
                previousTimeRight[leg[0] / 3][2] = millis();
            }
        }
    } else if (strcmp(order, "lift") == 0) {
        // Lift Forward
        float x_L_lift_F = -path_radius * cos(i) * cos(leg_angle);
        float y_L_lift_F = -path_radius * cos(i) * sin(leg_angle);
        float z_L_lift_F = -path_radius * sin(i);

        float x_L_T = -(-249.0f * tan(M_PI / 18.0) + 2.0f * 249.0f * k * tan(M_PI / 18.0));
        float y_L_T = 0.0f;

        // Calculate final positions
        x_L = path_center[0] + direction_vector[1] * x_L_lift_F + direction_vector[0] * x_L_T;
        y_L = path_center[1] + direction_vector[1] * y_L_lift_F + fabs(direction_vector[0] * y_L_T);
        z_L = path_center[2] - 1.5f * path_radius * sin(i);

        // Calculate angles and set servos
        float L_angles[3];
        calculateAngles(x_L, y_L, z_L, L_angles);


        if (strcmp(side, "left") == 0) {
            if (millis() - previousTimeLeft[leg[0] / 3][0] >= step_interval) {
                setServoAngle(pca1, leg[0], L_angles[0] + O[0]);
                previousTimeLeft[leg[0] / 3][0] = millis();
            }
            if (millis() - previousTimeLeft[leg[0] / 3][1] >= step_interval) {
                setServoAngle(pca1, leg[1], L_angles[1] + O[1]);
                previousTimeLeft[leg[0] / 3][1] = millis();
            }
            if (millis() - previousTimeLeft[leg[0] / 3][2] >= step_interval) {
                setServoAngle(pca1, leg[2], L_angles[2] + O[2] - 90);
                previousTimeLeft[leg[0] / 3][2] = millis();
            }
        } else if (strcmp(side, "right") == 0) {
            if (millis() - previousTimeRight[leg[0] / 3][0] >= step_interval) {
                setServoAngle(pca2, leg[0], L_angles[0] + O[0]);
                previousTimeRight[leg[0] / 3][0] = millis();
            }
            if (millis() - previousTimeRight[leg[0] / 3][1] >= step_interval) {
                setServoAngle(pca2, leg[1], -L_angles[1] + O[1]);
                previousTimeRight[leg[0] / 3][1] = millis();
            }
            if (millis() - previousTimeRight[leg[0] / 3][2] >= step_interval) {
                setServoAngle(pca2, leg[2], -L_angles[2] + 90 + O[2]);
                previousTimeRight[leg[0] / 3][2] = millis();
            }
        }
    }
}

void getUnitVector(const float* realVector, float* unitVector) {
    // Calculate the magnitude of the vector
    float magnitude = sqrt(pow(realVector[0], 2) + pow(realVector[1], 2));
    
    if (magnitude > 200) {
        unitVector[0] = -realVector[0] / magnitude;
        unitVector[1] = -realVector[1] / magnitude;
    } else {
        unitVector[0] = 0.0;
        unitVector[1] = 0.0;
    }
}

void stand() {
    float speed = 400.0f;
    float pathResolution;
    float resArray[2];
    getResFromSpeed(speed, resArray);
    pathResolution = resArray[0];
    // Target position for all legs
    float x_p = path_center_L2[0];
    float y_p = path_center_L2[1];
    float z_p = path_center_L2[2];

    // Set the position for each leg directly
    standbyPosition(path_center_L1[0], path_center_L1[1], path_center_L1[2], Left1, "left", L1o);
    standbyPosition(path_center_L2[0], path_center_L2[1], path_center_L2[2], Left2, "left", L2o);
    standbyPosition(path_center_L3[0], path_center_L3[1], path_center_L3[2], Left3, "left", L3o);
    standbyPosition(path_center_R1[0], path_center_R1[1], path_center_R1[2], Right1, "right", R1o);
    standbyPosition(path_center_R3[0], path_center_R3[1], path_center_R3[2], Right3, "right", R3o);
    standbyPosition(path_center_R2[0], path_center_R2[1], path_center_R2[2], Right2, "right", R2o);
}


void preWalk() {
    const float speed = 1100.0;
    float path_resolution;
    float resArray[2];
    getResFromSpeed(speed, resArray);
    path_resolution = resArray[0];
    const int resolution_half = (int)(path_resolution / 2);
    float i2_values[resolution_half];
    float j2_values[resolution_half];
    float k2_values[resolution_half];

    linspaceEmulation(0, M_PI, resolution_half, i2_values);
    linspaceEmulation(0, path_radius, resolution_half, j2_values);
    linspaceEmulation(0.01, 1.0, resolution_half, k2_values);

    unsigned long previousTime = 0;
    unsigned long previousTime2 = 0;

    for (int idx = 0; idx < resolution_half; ++idx) {
        processData(receivedData, ackPayload);
        directionVector[0] = receivedData.joystick[0];
        directionVector[1] = receivedData.joystick[1];

        float direction_vector[2];
        getUnitVector(directionVector, direction_vector);
        float magnitude = sqrt(direction_vector[0] * direction_vector[0] + direction_vector[1] * direction_vector[1]);
        
        if (magnitude > 0) 
        {
            float i = i2_values[idx];
            float j = j2_values[idx];
            float k = k2_values[idx];

            // Calculate dynamic sleep time
            float error = fabs((M_PI / 2) - i);
            float gain = 1.0;
            unsigned long sleep_time = (unsigned long)(step_interval * (1 + gain * error));

            if (millis() - previousTime >= sleep_time) {
              // Execute Pre_gait for all legs
              preGait(Left1, L1o, path_center_L1, path_radius, direction_vector, angle_L1, hex_radius, "drag", "left", i, j, k);
              preGait(Left2, L2o, path_center_L2, path_radius, direction_vector, angle_L2, hex_radius, "lift", "left", i, j, k);
              preGait(Left3, L3o, path_center_L3, path_radius, direction_vector, angle_L3, hex_radius, "drag", "left", i, j, k);

              preGait(Right1, R1o, path_center_R1, path_radius, direction_vector, angle_R1, hex_radius, "lift", "right", i, j, k);
              preGait(Right2, R2o, path_center_R2, path_radius, direction_vector, angle_R2, hex_radius, "drag", "right", i, j, k);
              preGait(Right3, R3o, path_center_R3, path_radius, direction_vector, angle_R3, hex_radius, "lift", "right", i, j, k);

              previousTime = millis();
            }
        }
        else {
          stand();
          break;
        }
        // delay(sleep_time);
        }

        for (int idx = 0; idx < resolution_half; ++idx) {
          processData(receivedData, ackPayload);
          directionVector[0] = receivedData.joystick[0];
          directionVector[1] = receivedData.joystick[1];

          float direction_vector[2];
          getUnitVector(directionVector, direction_vector);
          float magnitude = sqrt(direction_vector[0] * direction_vector[0] + direction_vector[1] * direction_vector[1]);
          if (magnitude > 0) 
         {
            float i = i2_values[idx];
            float j = j2_values[idx];
            float k = k2_values[idx];

            // Calculate dynamic sleep time
            float error = fabs((M_PI / 2) - i);
            float gain = 10.0;
            unsigned long sleep_time = (unsigned long)(step_interval * (1 + gain * error));

            if (millis() - previousTime2 >= sleep_time) {
              preGait(Left1, L1o, path_center_L1, path_radius, direction_vector, angle_L1, hex_radius, "lift", "left", i, j, k);
              preGait(Left2, L2o, path_center_L2, path_radius, direction_vector, angle_L2, hex_radius, "drag", "left", i, j, k);
              preGait(Left3, L3o, path_center_L3, path_radius, direction_vector, angle_L3, hex_radius, "lift", "left", i, j, k);

              preGait(Right1, R1o, path_center_R1, path_radius, direction_vector, angle_R1, hex_radius, "drag", "right", i, j, k);
              preGait(Right2, R2o, path_center_R2, path_radius, direction_vector, angle_R2, hex_radius, "lift", "right", i, j, k);
              preGait(Right3, R3o, path_center_R3, path_radius, direction_vector, angle_R3, hex_radius, "drag", "right", i, j, k);
              
              previousTime2 = millis();
            }
         }
         else {
          stand();
          break;
        }
            // delay(sleep_time);
        }
    
}

void walk() {
    const float speed = 1100.0;
    float path_resolution;
    float resArray[2];
    getResFromSpeed(speed, resArray);
    path_resolution = resArray[0];
    int resolution = (int)(path_resolution);
    float i_values[resolution];
    float j_values[resolution];
    float k_values[resolution];

    linspaceEmulation(0.0f, M_PI, resolution, i_values);                 // Points for lifting
    linspaceEmulation(0.0f, 2.0f * path_radius, resolution, j_values);  // Points for dragging
    linspaceEmulation(0.01f, 1.0f, resolution, k_values);               // Points for turning

    unsigned long previousTime = 0;
    unsigned long previousTime2 = 0;

    for (int idx = 0; idx < resolution; ++idx) {
        processData(receivedData, ackPayload);
        directionVector[0] = receivedData.joystick[0];
        directionVector[1] = receivedData.joystick[1];

        float direction_vector[2];
        getUnitVector(directionVector, direction_vector);
        float magnitude = sqrt(direction_vector[0] * direction_vector[0] + direction_vector[1] * direction_vector[1]);
        
        if (magnitude > 0) 
        {
            float i = i_values[idx];
            float j = j_values[idx];
            float k = k_values[idx];

            // Calculate dynamic sleep time
            float error = fabs((M_PI / 2) - i);
            float gain = 1.0;
            unsigned long sleep_time = (unsigned long)(step_interval * (1 + gain * error));

            if (millis() - previousTime >= sleep_time) {
              // Execute Pre_gait for all legs
              gait(Left1, L1o, path_center_L1, path_radius, direction_vector, angle_L1, hex_radius, "drag", "left", i, j, k);
              gait(Left2, L2o, path_center_L2, path_radius, direction_vector, angle_L2, hex_radius, "lift", "left", i, j, k);
              gait(Left3, L3o, path_center_L3, path_radius, direction_vector, angle_L3, hex_radius, "drag", "left", i, j, k);

              gait(Right1, R1o, path_center_R1, path_radius, direction_vector, angle_R1, hex_radius, "lift", "right", i, j, k);
              gait(Right2, R2o, path_center_R2, path_radius, direction_vector, angle_R2, hex_radius, "drag", "right", i, j, k);
              gait(Right3, R3o, path_center_R3, path_radius, direction_vector, angle_R3, hex_radius, "lift", "right", i, j, k);

              previousTime = millis();
            }
        }
        else {
          stand();
          break;
        }
        // delay(sleep_time);
        }

        for (int idx = 0; idx < resolution; ++idx) {
          processData(receivedData, ackPayload);
          directionVector[0] = receivedData.joystick[0];
          directionVector[1] = receivedData.joystick[1];

          float direction_vector[2];
          getUnitVector(directionVector, direction_vector);
          float magnitude = sqrt(direction_vector[0] * direction_vector[0] + direction_vector[1] * direction_vector[1]);
          if (magnitude > 0) 
         {
            float i = i_values[idx];
            float j = j_values[idx];
            float k = k_values[idx];

            // Calculate dynamic sleep time
            float error = fabs((M_PI / 2) - i);
            float gain = 10.0;
            unsigned long sleep_time = (unsigned long)(step_interval * (1 + gain * error));

            if (millis() - previousTime2 >= sleep_time) {
                gait(Left1, L1o, path_center_L1, path_radius, direction_vector, angle_L1, hex_radius, "lift", "left", i, j, k);
                gait(Left2, L2o, path_center_L2, path_radius, direction_vector, angle_L2, hex_radius, "drag", "left", i, j, k);
                gait(Left3, L3o, path_center_L3, path_radius, direction_vector, angle_L3, hex_radius, "lift", "left", i, j, k);
                
                gait(Right1, R1o, path_center_R1, path_radius, direction_vector, angle_R1, hex_radius, "drag", "right", i, j, k);
                gait(Right2, R2o, path_center_R2, path_radius, direction_vector, angle_R2, hex_radius, "lift", "right", i, j, k);
                gait(Right3, R3o, path_center_R3, path_radius, direction_vector, angle_R3, hex_radius, "drag", "right", i, j, k);
              
              previousTime2 = millis();
            }
         }
         else {
          stand();
          break;
        }
            // delay(sleep_time);
        }
    
}

void standby() {
    float speed = 400.0f;
    float pathResolution;
    float resArray[2];
    getResFromSpeed(speed, resArray);
    pathResolution = resArray[0];
    const int res = (int)(pathResolution);  // Resolution as an integer
    float z_values[res];
    linspaceEmulation(0.0f, Robot_height, res, z_values);

    for (int i = 0; i < res; ++i) {
        float z_p = z_values[i];
        float x_p = 0.0f;
        float y_p = path_center_L2[1];

        // Set the position for each leg
        standbyPosition(x_p, y_p, z_p, Left1, "left", L1o);
        standbyPosition(x_p, y_p, z_p, Left2, "left", L2o);
        standbyPosition(x_p, y_p, z_p, Left3, "left", L3o);
        standbyPosition(x_p, y_p, z_p, Right1, "right", R1o);
        standbyPosition(x_p, y_p, z_p, Right3, "right", R3o);
        standbyPosition(x_p, y_p, z_p, Right2, "right", R2o);
    }
}

void sit() {
    float speed = 400.0;
    float path_resolution;
    float resArray[2];
    // Calculate step interval and resolution based on speed
    getResFromSpeed(speed, resArray);
    path_resolution = resArray[0];
    // Prepare z-values for the downward movement
    float z_values[(int)path_resolution];
    linspaceEmulation(Robot_height, -100.0, (int)(path_resolution), z_values);

    // Iterate through z-values to lower the robot
    for (int i = 0; i < (int)(path_resolution); ++i) {
        float z_p = z_values[i];
        float x_p = 0.0;
        float y_p = path_center_L2[1];

        // Set legs to the new position
        standbyPosition(x_p, y_p, z_p, Left1, "left", L1o);
        standbyPosition(x_p, y_p, z_p, Left2, "left", L2o);
        standbyPosition(x_p, y_p, z_p, Left3, "left", L3o);
        standbyPosition(x_p, y_p, z_p, Right1, "right", R1o);
        standbyPosition(x_p, y_p, z_p, Right3, "right", R3o);
        standbyPosition(x_p, y_p, z_p, Right2, "right", R2o);
    }
}

#endif // HEX_MOVE_H
