# Omni-Rvoer
The code for omni rover 
 // Autonomous Omni-Directional Rover Control System with IMU, Color Sensor, 
   and Ultrasonic integration 
// Includes Position Tracking, Heading Estimation, and Motor Dynamics Control Model

#include <cmath>
#include <complex>
using std::complex;

struct Vector3 {
    float x;   // forward velocity (m/s)
    float y;   // lateral velocity (m/s)
    float wz;  // angular velocity (rad/s)
};

// Physical and Electrical Constants
const float R = 0.05;             // Wheel radius (meters)
const float Kb = 0.01;            // Back EMF constant
const float L = 0.2;              // Robot length (meters)
const float W = 0.2;              // Robot width (meters)
const float Kp = 1.0;             // Proportional control gain
const float inductance = 0.01;    // Motor inductance (H)
const float resistance = 1.0;     // Motor resistance (Ohms)
const float dt = 0.05;            // Loop time step (seconds)
const float spikeThreshold = 20.0; // Ultrasonic noise filter threshold (cm)
const float OBSTACLE_THRESHOLD = 25.0; // Distance to trigger avoidance (cm)

// Sensor Inputs
float ultrasonicDistance[4];         // Ultrasonic 
distances [Front, Right, Back, Left] (cm)
float lastUltrasonicDistance[4] = {0, 0, 0, 0};
float filteredSpeed[4] = {0, 0, 0, 0};
bool ultrasonicValid[4] = {false, false, false, false};

float gyroZ;         // Gyro Z-axis rate (rad/s)
float headingDeg = 0.0f; // Integrated heading (degrees)
float accX, accY, accZ; // Optional accelerometer data

// Color Sensor Inputs
int colorSensorLeft;
int colorSensorRight;

// Motor Control Inputs
float e[4];       // Voltage control error for each motor
float v[4];       // Reference/control voltage
float di_dt[4];   // Derivative of motor current
float ia[4];      // Armature current

// Grid Position Tracking (complex plane)
int realAxis = 0;     // Horizontal (real) position
int imagAxis = 0;     // Vertical (imaginary) position
int prevColorLeft = 0, prevColorRight = 0;
complex<float> targetPosition(5, 3); // Grid coordinate goal

// Update heading angle from IMU gyro
void updateIMUHeading() {
    headingDeg += gyroZ * dt * 180.0f / M_PI;
    if (headingDeg > 180.0f) headingDeg -= 360.0f;
    if (headingDeg < -180.0f) headingDeg += 360.0f;
}

// Compute angle from current position to target (degrees)
float computeAngleToTarget() {
    complex<float> currentPosition(realAxis, imagAxis);
    complex<float> toTarget = targetPosition - currentPosition;
    return atan2(toTarget.imag(), toTarget.real()) * 180.0f / M_PI;
}

// Calculate signed heading difference (target - current)
float computeHeadingError() {
    float targetAngle = computeAngleToTarget();
    float diff = targetAngle - headingDeg;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// Compute body velocity using motor dynamics control equation
Vector3 computeBodyVelocity() {
    float input[4];
    for (int i = 0; i < 4; ++i) {
        input[i] = Kp * e[i] + v[i] - inductance * di_dt[i] - resistance * ia[i];
    }
    float factor = R / (4.0f * Kb);
    float LplusW = L + W;

    Vector3 result;
    result.x = factor * (input[0] + input[1] + input[2] + input[3]);
    result.y = factor * (input[0] - input[1] - input[2] + input[3]);
    result.wz = factor * (-input[0] + input[1] - input[2] + input[3]) / LplusW;
    return result;
}

// Update grid position from color sensor transitions
void updatePositionFromColorSensors(int leftColor, int rightColor) {
    if (prevColorLeft == 2 && leftColor == 1) imagAxis++;
    else if (prevColorLeft == 1 && leftColor == 2) imagAxis--;
    if (prevColorRight == 3 && rightColor == 1) realAxis++;
    else if (prevColorRight == 1 && rightColor == 3) realAxis--;
    if ((prevColorLeft == 3 && prevColorRight == 2) && (leftColor == 4 && rightColor == 4)) {
        realAxis++; imagAxis++;
    }
    if ((prevColorLeft == 1 && prevColorRight == 1) && (leftColor == 4 && rightColor == 4)) {
        realAxis--; imagAxis--;
    }
    prevColorLeft = leftColor;
    prevColorRight = rightColor;
}

// Check if robot has reached the target
bool isAtTarget() {
    return realAxis == static_cast<int>(targetPosition.real()) &&
           imagAxis == static_cast<int>(targetPosition.imag());
}

// Motion Types for Omni Rover
enum MotionType {
    FORWARD, BACKWARD, LEFT, RIGHT, ROTATE_CW, ROTATE_CCW,
    FORWARD_LEFT, FORWARD_RIGHT, BACKWARD_LEFT, BACKWARD_RIGHT
};

// Speed Profile
enum SpeedClass { SLOW, FAST };

// Obstacle Avoidance FSM
enum AvoidanceState {
    NORMAL, AVOID_LEFT, AVOID_BACK,
    WAITING_LEFT_CLEAR, WAITING_BACK_CLEAR
};

AvoidanceState avoidanceState = NORMAL;

// Get motor reference voltages for a given motion type and speed
void getReferenceVoltages(MotionType motion, SpeedClass speed, float vRef[4]) {
    float base = (speed == FAST) ? 10.0f : 5.0f;
    switch (motion) {
        case FORWARD: vRef[0] = vRef[1] = vRef[2] = vRef[3] = base; break;
        case BACKWARD: vRef[0] = vRef[1] = vRef[2] = vRef[3] = -base; break;
        case LEFT: vRef[0] = base; vRef[1] = -base; vRef[2] = -base; vRef[3] = base; break;
        case RIGHT: vRef[0] = -base; vRef[1] = base; vRef[2] = base; vRef[3] = -base; break;
        case ROTATE_CW: vRef[0] = -base; vRef[1] = base; vRef[2] = -base; vRef[3] = base; break;
        case ROTATE_CCW: vRef[0] = base; vRef[1] = -base; vRef[2] = base; vRef[3] = -base; break;
        case FORWARD_LEFT: vRef[0] = base; vRef[1] = 0; vRef[2] = 0; vRef[3] = base; break;
        case FORWARD_RIGHT: vRef[0] = 0; vRef[1] = base; vRef[2] = base; vRef[3] = 0; break;
        case BACKWARD_LEFT: vRef[0] = -base; vRef[1] = 0; vRef[2] = 0; vRef[3] = -base; break;
        case BACKWARD_RIGHT: vRef[0] = 0; vRef[1] = -base; vRef[2] = -base; vRef[3] = 0; break;
    }
}

// Obstacle Avoidance Logic using Finite State Machine
void handleObstacleAvoidance() {
    updateIMUHeading();
    updatePositionFromColorSensors(colorSensorLeft, colorSensorRight);

    if (isAtTarget()) {
        for (int i = 0; i < 4; i++) v[i] = 0; // Stop motors
        return;
    }

    switch (avoidanceState) {
        case NORMAL:
            if (ultrasonicDistance[0] < OBSTACLE_THRESHOLD && ultrasonicValid[0]) {
                getReferenceVoltages(ROTATE_CCW, SLOW, v);
                avoidanceState = AVOID_LEFT;
            } else {
                float error = computeHeadingError();
                if (fabs(error) > 5) getReferenceVoltages(ROTATE_CCW, SLOW, v);
                else getReferenceVoltages(FORWARD, FAST, v);
            }
            break;

        case AVOID_LEFT:
            if (ultrasonicDistance[3] >= OBSTACLE_THRESHOLD && ultrasonicValid[3]) {
                getReferenceVoltages(FORWARD, SLOW, v);
                avoidanceState = WAITING_LEFT_CLEAR;
            } else {
                getReferenceVoltages(ROTATE_CCW, SLOW, v);
                avoidanceState = AVOID_BACK;
            }
            break;

        case AVOID_BACK:
            if (ultrasonicDistance[2] >= OBSTACLE_THRESHOLD && ultrasonicValid[2]) {
                getReferenceVoltages(FORWARD, SLOW, v);
                avoidanceState = WAITING_BACK_CLEAR;
            }
            break;

        case WAITING_LEFT_CLEAR:
            if (ultrasonicDistance[3] >= OBSTACLE_THRESHOLD &&
                ultrasonicDistance[0] >= OBSTACLE_THRESHOLD) {
                float error = computeHeadingError();
                if (fabs(error) > 5) getReferenceVoltages(ROTATE_CW, SLOW, v);
                else {
                    getReferenceVoltages(FORWARD, FAST, v);
                    avoidanceState = NORMAL;
                }
            }
            break;

        case WAITING_BACK_CLEAR:
            if (ultrasonicDistance[3] >= OBSTACLE_THRESHOLD) {
                float error = computeHeadingError();
                if (fabs(error) > 5) getReferenceVoltages(ROTATE_CCW, SLOW, v);
                else {
                    getReferenceVoltages(FORWARD, FAST, v);
                    avoidanceState = NORMAL;
                }
            }
            break;
    }
}
