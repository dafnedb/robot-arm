// =======================================================
// ARDUINO MEGA 2560 - 3 Steppers + 2 Servos
// =======================================================
#include <Arduino.h>
#include <Servo.h>

// ===== RAMPS 1.6 PINS - STEPPERS =====
#define X_STEP 54 // NEMA 1 (Joy 1 X)
#define X_DIR 55
#define X_EN 38

#define Y_STEP 60 // NEMA 2 (Joy 1 Y)
#define Y_DIR 61
#define Y_EN 56

#define Z_STEP 46 // NEMA 3 (Joy 2 Y)
#define Z_DIR 48
#define Z_EN 62

// ===== SERVO PINS =====
const int PIN_PINZA = 11;
const int PIN_SERVO2 = 6;

// ===== MOVEMENT LIMITS =====
const long LIM_X = 178;
const long LIM_Y_DER = 178;
const long LIM_Y_IZQ = -1066;
const long LIM_Z_MAX = 5000;
const long LIM_Z_MIN = -5000;

// ===== POSITION AND CONTROL VARIABLES =====
int vx1 = 0, vy1 = 0;
int vx2 = 0, vy2 = 0;
int potValue = 0;

long posX = 0, posY = 0, posZ = 0;
unsigned long lastStepX = 0, lastStepY = 0, lastStepZ = 0;
unsigned long lastCmdTime = 0;
// Servo 2 angle control variable
float wristAngle = 90.0;

const unsigned long WATCHDOG_TIMEOUT = 500;
const int MAX_SPEED = 200;

// ===== SERVO OBJECTS =====
Servo clamp;
Servo wrist;

// Function prototypes for better organization
void stepMotor(int stepPin, int dirPin, int speed,
               unsigned long &lastStep,
               long &posCounter,
               long minLimit,
               long maxLimit);

void setup()
{
    // Initialize serial communication for debugging and command reception
    // - Serial for debugging output
    // - Serial1 for receiving commands from the main controller
    Serial.begin(115200);
    Serial1.begin(115200);
    // Set pin modes for stepper control pins
    pinMode(X_STEP, OUTPUT);
    pinMode(X_DIR, OUTPUT);
    pinMode(X_EN, OUTPUT);
    pinMode(Y_STEP, OUTPUT);
    pinMode(Y_DIR, OUTPUT);
    pinMode(Y_EN, OUTPUT);
    pinMode(Z_STEP, OUTPUT);
    pinMode(Z_DIR, OUTPUT);
    pinMode(Z_EN, OUTPUT);
    // Disable all stepper drivers at startup
    digitalWrite(X_EN, LOW);
    digitalWrite(Y_EN, LOW);
    digitalWrite(Z_EN, LOW);
    // Initialize servos
    clamp.attach(PIN_PINZA);
    wrist.attach(PIN_SERVO2);
    // Set initial positions for servos
    clamp.write(90);
    wrist.write((int)wristAngle); // Initialize wrist servo to 90 degrees
    Serial.println("Sistema Listo");
}

void loop()
{
    // Check for incoming commands from the ESP32 via Serial1
    if (Serial1.available())
    {
        String input = Serial1.readStringUntil('\n');
        int startIdx = input.indexOf('<');
        int endIdx = input.indexOf('>');

        if (startIdx != -1 && endIdx != -1)
        {
            String data = input.substring(startIdx + 1, endIdx);
            int t_vx1, t_vy1, t_vx2, t_vy2, t_pot;

            if (sscanf(data.c_str(), "%d,%d,%d,%d,%d", &t_vx1, &t_vy1, &t_vx2, &t_vy2, &t_pot) == 5)
            {
                vx1 = constrain(t_vx1, -MAX_SPEED, MAX_SPEED); // Speed value for stepper 1 (X-axis)
                vy1 = constrain(t_vy1, -MAX_SPEED, MAX_SPEED); // Speed value for stepper 2 (Y-axis)
                vx2 = constrain(t_vx2, -MAX_SPEED, MAX_SPEED); // Speed value for wrist servo (incremental control)
                vy2 = constrain(t_vy2, -MAX_SPEED, MAX_SPEED); // Speed value for stepper 3 (Z-axis)
                potValue = t_pot;                              // Potentiometer value for clamp servo (0-180)
                // Update the last command time for the watchdog timer
                lastCmdTime = millis();
                // Set clamp servo position based on potentiometer value
                clamp.write(potValue);
                // Incremental control for wrist servo based on vx2 value
                if (vx2 != 0)
                {
                    // Increment wrist angle based on vx2 speed, with a small scaling factor for smooth control
                    // The wrist angle is updated incrementally, allowing for fine control over the servo position.
                    // The angle is constrained to the range of 0 to 180 degrees to prevent damage to the servo.
                    wristAngle += (vx2 * 0.005);
                    wristAngle = constrain(wristAngle, 0.0, 180.0);
                    wrist.write((int)wristAngle);
                }
            }
        }
    }

    // ===== WATCHDOG TIMER =====
    // If no command is received within the WATCHDOG_TIMEOUT period, stop all movements by setting speeds to zero.
    // This safety feature ensures that the robot does not continue moving indefinitely if communication is lost.
    if (millis() - lastCmdTime > WATCHDOG_TIMEOUT)
    {
        vx1 = 0;
        vy1 = 0;
        vy2 = 0;
        vx2 = 0;
    }

    // ===== STEP MOTOR CONTROL =====
    stepMotor(X_STEP, X_DIR, vx1, lastStepX, posX, -LIM_X, LIM_X);
    stepMotor(Y_STEP, Y_DIR, vy1, lastStepY, posY, LIM_Y_IZQ, LIM_Y_DER);
    stepMotor(Z_STEP, Z_DIR, vy2, lastStepZ, posZ, LIM_Z_MIN, LIM_Z_MAX);
}

/**
 * Controls a stepper motor based on the provided speed and direction.
 * The function checks the current position against defined limits to prevent overtravel.
 * It uses a timing mechanism to control the stepping frequency based on the speed value, allowing for smooth acceleration and deceleration of the motor.
 */

void stepMotor(int stepPin, int dirPin, int speed, unsigned long &lastStep, long &posCounter, long maxLimit, long minLimit)
{
    if (speed == 0)
        return;

    bool direction = (speed > 0);

    if (direction && posCounter >= minLimit)
        return;
    if (!direction && posCounter <= maxLimit)
        return;

    digitalWrite(dirPin, direction);

    unsigned long interval = map(abs(speed), 1, MAX_SPEED, 3000, 300);

    if (micros() - lastStep >= interval)
    {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(stepPin, LOW);

        if (direction)
            posCounter++;
        else
            posCounter--;
        lastStep = micros();
    }
}