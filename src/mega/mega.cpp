// =======================================================
// ARDUINO MEGA 2560 - 3 Steppers + 2 Servos + HOME VIRTUAL
// =======================================================
#include <Arduino.h>
#include <Servo.h>

// ===== RAMPS 1.6 PINS - STEPPERS =====
#define X_STEP 54 
#define X_DIR 55
#define X_EN 38
#define Y_STEP 60 
#define Y_DIR 61
#define Y_EN 56
#define Z_STEP 46 
#define Z_DIR 48
#define Z_EN 62

// ===== PIN SENSOR (X-MIN) =====
#define X_MIN_PIN 3 

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
int resetSignal = 0; // Sexto valor recibido: reset/home

long posX = 0, posY = 0, posZ = 0;
unsigned long lastStepX = 0, lastStepY = 0, lastStepZ = 0;
unsigned long lastCmdTime = 0;
float wristAngle = 90.0;
bool haciendoHome = false; // Bloquea control manual durante el home

const unsigned long WATCHDOG_TIMEOUT = 500;
const int MAX_SPEED = 200;

Servo clamp;
Servo wrist;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200); // Comunicación con ESP32 Receptor

    pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT); pinMode(X_EN, OUTPUT);
    pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT); pinMode(Y_EN, OUTPUT);
    pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT); pinMode(Z_EN, OUTPUT);

    pinMode(X_MIN_PIN, INPUT_PULLUP); // Sensor Makerbot

    digitalWrite(X_EN, LOW);
    digitalWrite(Y_EN, LOW);
    digitalWrite(Z_EN, LOW);

    clamp.attach(PIN_PINZA);
    wrist.attach(PIN_SERVO2);
    clamp.write(90);
    wrist.write((int)wristAngle);
    
    Serial.println("Mega Lista - Sistema de Home Virtual Configurado");
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
                clamp.write(potValue);
                
                if (vx2 != 0) {
                    wristAngle += (vx2 * 0.005);
                    wristAngle = constrain(wristAngle, 0.0, 180.0);
                    wrist.write((int)wristAngle);
                }
            }
        }
    }

    // 2. ACTIVAR HOME SI SE RECIBE SEÑAL DE RESET
    if (resetSignal == 1 && !haciendoHome) {
        ejecutarHomeVirtualX();
    }

    // 3. MOVIMIENTO NORMAL (SOLO SI NO ESTÁ EN HOME)
    if (!haciendoHome) {
        if (millis() - lastCmdTime > WATCHDOG_TIMEOUT) {
            vx1 = 0; vy1 = 0; vy2 = 0; vx2 = 0;
        }

        stepMotor(X_STEP, X_DIR, vx1, lastStepX, posX, -LIM_X, LIM_X);
        stepMotor(Y_STEP, Y_DIR, vy1, lastStepY, posY, LIM_Y_IZQ, LIM_Y_DER);
        stepMotor(Z_STEP, Z_DIR, vy2, lastStepZ, posZ, LIM_Z_MIN, LIM_Z_MAX);
    }
}

// FUNCIÓN DE HOME: BUSCA EL EXTREMO Y SE DESPLAZA AL CENTRO
void ejecutarHomeVirtualX() {
    haciendoHome = true;
    Serial.println("HOME: Iniciando búsqueda...");

    // 1. BUSCAR EL SENSOR
    digitalWrite(X_DIR, LOW); 
    while (digitalRead(X_MIN_PIN) == HIGH) {
        digitalWrite(X_STEP, HIGH);
        delayMicroseconds(1000); 
        digitalWrite(X_STEP, LOW);
        delayMicroseconds(1000);
    }
    
    // 2. CONFIRMACIÓN Y PAUSA (Evita rebote mecánico)
    delay(200); 

    // 3. MOVER 10 GRADOS (178 pasos para 32uS)
    Serial.println("Sensor tocado. Moviendo 10 grados...");
    digitalWrite(X_DIR, HIGH); 
    for (int i = 0; i < 178; i++) {
        digitalWrite(X_STEP, HIGH);
        delayMicroseconds(1200); 
        digitalWrite(X_STEP, LOW);
        delayMicroseconds(1200);
    }
    
    // 4. LIMPIEZA CRUCIAL PARA EVITAR REBOTES
    posX = 178; 
    resetSignal = 0; // Borramos la variable local
    
    // Vaciamos el buffer serial: Borra cualquier "1" que haya quedado pendiente de la ESP32
    while(Serial1.available() > 0) {
        Serial1.read();
    }

    haciendoHome = false;
    Serial.println("HOME FINALIZADO: Sistema en espera.");
}

void stepMotor(int stepPin, int dirPin, int speed, unsigned long &lastStep, long &posCounter, long minLimit, long maxLimit) {
    if (speed == 0) return;
    bool direction = (speed > 0);
    if (direction && posCounter >= maxLimit) return;
    if (!direction && posCounter <= minLimit) return;

    digitalWrite(dirPin, direction);
    unsigned long interval = map(abs(speed), 1, MAX_SPEED, 3000, 300);

    if (micros() - lastStep >= interval) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(stepPin, LOW);
        if (direction) posCounter++; else posCounter--;
        lastStep = micros();
    }
}
