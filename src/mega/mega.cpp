// =======================================================
// ARDUINO MEGA 2560 - 3 Steppers + 2 Servos + HOME VIRTUAL (CORREGIDO)
// =======================================================
#include <Arduino.h>
#include <Servo.h>

// ===== RAMPS 1.6 PINS - STEPPERS =====
#define X_STEP 54   // NEMA 1 (Joy 1 X)
#define X_DIR 55
#define X_EN 38
#define Y_STEP 60 // NEMA 2 (Joy 1 Y)
#define Y_DIR 61
#define Y_EN 56
#define Z_STEP 46 // NEMA 3 (Joy 2 Y)
#define Z_DIR 48
#define Z_EN 62

// ===== PIN SENSOR (X-MIN) =====
#define X_MIN_PIN 3 
#define Y_MIN_PIN 14 // Pin correspondiente a Y-MIN en RAMPS

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
int vx1 = 0, vy1 = 0, vx2 = 0, vy2 = 0, potValue = 0;
int resetSignal = 0; 

long posX = 0, posY = 0, posZ = 0;
unsigned long lastStepX = 0, lastStepY = 0, lastStepZ = 0;
unsigned long lastCmdTime = 0;

// Servo 2 angle control variable
float wristAngle = 90.0;
bool haciendoHome = false;  // Bloquea control manual durante el home

const unsigned long WATCHDOG_TIMEOUT = 500;
const int MAX_SPEED = 200;

// ===== SERVO OBJECTS =====
Servo clamp;
Servo wrist;

// Function prototypes for better organization
void stepMotor(int stepPin, int dirPin, int speed, unsigned long &lastStep, long &posCounter, long minLimit, long maxLimit);
void ejecutarHomeVirtualX();

void setup() {
  // Initialize serial communication for debugging and command reception
  // - Serial for debugging output
  // - Serial1 for receiving commands from the main controller
    Serial.begin(115200);
    Serial1.begin(115200); 

// Set pin modes for stepper control pins
    pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT); pinMode(X_EN, OUTPUT);
    pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT); pinMode(Y_EN, OUTPUT);
    pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT); pinMode(Z_EN, OUTPUT);
    pinMode(X_MIN_PIN, INPUT_PULLUP);
    pinMode(Y_MIN_PIN, INPUT_PULLUP);

// Disable all stepper drivers at startup
    digitalWrite(X_EN, LOW);
    digitalWrite(Y_EN, LOW);
    digitalWrite(Z_EN, LOW);

// Initialize servos
    clamp.attach(PIN_PINZA);
    wrist.attach(PIN_SERVO2);
    clamp.write(90);
    wrist.write((int)wristAngle);  // Initialize wrist servo to 90 degrees
    
    Serial.println("Sistema Iniciado");
}

void loop() {
    // 1. RECEPCIÓN DE COMANDOS (6 VALORES: <vx1,vy1,vx2,vy2,pot,reset>)
    if (Serial1.available()) {
        String input = Serial1.readStringUntil('\n');
        int startIdx = input.indexOf('<');
        int endIdx = input.indexOf('>');

        if (startIdx != -1 && endIdx != -1) {
            String data = input.substring(startIdx + 1, endIdx);
            int t_vx1, t_vy1, t_vx2, t_vy2, t_pot, t_res;

            if (sscanf(data.c_str(), "%d,%d,%d,%d,%d,%d", &t_vx1, &t_vy1, &t_vx2, &t_vy2, &t_pot, &t_res) == 6) {
                vx1 = constrain(t_vx1, -MAX_SPEED, MAX_SPEED);
                vy1 = constrain(t_vy1, -MAX_SPEED, MAX_SPEED);
                vx2 = constrain(t_vx2, -MAX_SPEED, MAX_SPEED);
                vy2 = constrain(t_vy2, -MAX_SPEED, MAX_SPEED);
                potValue = t_pot;
                resetSignal = t_res; 
                lastCmdTime = millis();

                if (!haciendoHome) {
                  // Set clamp servo position based on potentiometer value
                    clamp.write(potValue);
                    // Incremental control for wrist servo based on vx2 value
                    // The wrist angle is updated incrementally, allowing for fine control over the servo position.
                     // The angle is constrained to the range of 0 to 180 degrees to prevent damage to the servo.
                    if (vx2 != 0) { 
                    // Increment wrist angle based on vx2 speed, with a small scaling factor for smooth control
                        wristAngle += (vx2 * 0.05); // Ajustado factor de escala
                        wristAngle = constrain(wristAngle, 0.0, 180.0);
                        wrist.write((int)wristAngle);
                    }
                }
            }
        }
    }

    // 2. ===== WATCHDOG TIMER =====
    // If no command is received within the WATCHDOG_TIMEOUT period, stop all movements by setting speeds to zero.
    // This safety feature ensures that the robot does not continue moving indefinitely if communication is lost.
    if (millis() - lastCmdTime > WATCHDOG_TIMEOUT) {
        vx1 = 0; vy1 = 0; vy2 = 0; vx2 = 0;
    }

    // 3. LÓGICA DE HOME
    if (resetSignal == 1 && !haciendoHome) {
        ejecutarHomeVirtual();
    }

    // 4. MOVIMIENTO MOTORES (Solo si no está haciendo Home)
    if (!haciendoHome) {
        stepMotor(X_STEP, X_DIR, vx1, lastStepX, posX, -LIM_X, LIM_X);
        stepMotor(Y_STEP, Y_DIR, vy1, lastStepY, posY, LIM_Y_IZQ, LIM_Y_DER);
        stepMotor(Z_STEP, Z_DIR, vy2, lastStepZ, posZ, LIM_Z_MIN, LIM_Z_MAX);
    }
} // <--- AQUÍ TERMINA EL LOOP

void ejecutarHomeVirtual() {
    haciendoHome = true;
    Serial.println("HOME: Iniciando...");

    digitalWrite(X_DIR, LOW); 
    while (digitalRead(X_MIN_PIN) == HIGH) {
        digitalWrite(X_STEP, HIGH);
        delayMicroseconds(1000); 
        digitalWrite(X_STEP, LOW);
        delayMicroseconds(1000);
    }
    
    delay(200); 

    Serial.println("Sensor tocado. Moviendo a offset...");
    digitalWrite(X_DIR, HIGH); 
    for (int i = 0; i < 178; i++) {
        digitalWrite(X_STEP, HIGH);
        delayMicroseconds(1200); 
        digitalWrite(X_STEP, LOW);
        delayMicroseconds(1200);
    }
    
    posX = 178;
    Serial.println("HOME Y: Buscando sensor...");
    digitalWrite(Y_DIR, LOW); // Asegúrate que LOW sea hacia el sensor
    while (digitalRead(Y_MIN_PIN) == HIGH) {
        digitalWrite(Y_STEP, HIGH); delayMicroseconds(1000);
        digitalWrite(Y_STEP, LOW);  delayMicroseconds(1000);
    }
    // Offset Y (Ajusta los pasos según tu mecánica)
    digitalWrite(Y_DIR, HIGH); 
    for (int i = 0; i < 200; i++) {
        digitalWrite(Y_STEP, HIGH); delayMicroseconds(1200);
        digitalWrite(Y_STEP, LOW);  delayMicroseconds(1200);
    }
    posY = 0; // O el valor que defina tu centro
    resetSignal = 0; 
    
    while(Serial1.available() > 0) { Serial1.read(); }

    haciendoHome = false;
    Serial.println("HOME FINALIZADO");
}

void stepMotor(int stepPin, int dirPin, int speed, unsigned long &lastStep, long &posCounter, long minLimit, long maxLimit) {
    if (speed == 0) return;
    
    bool direction = (speed > 0);
    
    // Bloqueo por límites
    if (direction && posCounter >= maxLimit) return;
    if (!direction && posCounter <= minLimit) return;

    unsigned long interval = map(abs(speed), 1, MAX_SPEED, 3000, 300);

    if (micros() - lastStep >= interval) {
        digitalWrite(dirPin, direction);
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(stepPin, LOW);
        
        if (direction) posCounter++; else posCounter--;
        lastStep = micros();
    }
}
