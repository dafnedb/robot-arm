/**
 * ==============================================================
 *  Control Remoto ESP32 → Robot vía UDP
 * ==============================================================
 *  Dispositivo: ESP32
 *  Comunicación: UDP binario (5 bytes)
 *  Frecuencia de envío: 50 Hz
 *
 *  Paquete enviado:
 *  ---------------------------------
 *  int16_t joyX  -> Velocidad eje X (-200 a 200)
 *  int16_t joyY  -> Velocidad eje Y (-200 a 200)
 *  uint8_t pot   -> Escala 0–255
 *  ---------------------------------
 *
 *  Características:
 *  - Deadzone configurable
 *  - Curva exponencial para control fino
 *  - Calibración automática del center
 *  - Reconexión WiFi automática
 *
 * ==============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// CONFIGURACIÓN CONEXIÓN
const char *ssid = "MTDS_ROBOT_AP";
const char *password = "robotseguro";
const char *udpAddress = "192.168.4.1";
const uint16_t udpPort = 4210;
// PIN DEL POTENCIÓMETRO
const int POT_PIN = 32;
// PINES JOYSTICK
const int JOY_X_PIN = 34;
const int JOY_Y_PIN = 35;

// OBJETOS
WiFiUDP udp;

// =======================================================
// CONFIGURACIÓN GENERAL
// =======================================================

// Velocidad máxima mapeada
const int MAX_SPEED = 200;
const int SEND_INTERVAL = 20; // 50 Hz

const int minX_Real = 0;
const int maxX_Real = 4095;
const int minY_Real = 0;
const int maxY_Real = 4095;
// Rango de deadzone en unidades ADC
const int DEADZONE = 30;
// EXPONENT para curva de control (1.0 = lineal, >1.0 = más suave cerca del center)
const float EXPONENT = 2.0;
// Variables para calibración automática del center
int centerX = 2048;
int centerY = 2048;
// Tiempo del último envío para control de frecuencia
unsigned long lastSend = 0;

// =======================================================
// DEFINICIÓN DEL PAQUETE BINARIO (8 BYTES EXACTOS)
// =======================================================

/**
 * Definición del paquete de datos que se enviará al robot.
 * Se utiliza #pragma pack para asegurar que no haya padding entre los campos,
 * garantizando que el tamaño del struct sea exactamente 5 bytes.
 *
 * El struct utiliza _attribute__((packed)) para reforzar esta garantía.
 */

#pragma pack(push, 1)
typedef struct __attribute__((packed))
{
  int16_t joyX; // value de velocidad en eje X (-200 a 200)
  int16_t joyY; // value de velocidad en eje Y (-200 a 200)
  uint8_t pot;  // value del potenciómetro mapeado a 0-255
} DataPacket;
#pragma pack(pop)
// Control de tamaño para asegurar que el struct mide exactamente 5 bytes
static_assert(sizeof(DataPacket) == 5, "El struct no mide 5 bytes");

// =======================================================
// PROCESAMIENTO JOYSTICK
// =======================================================

/**
 * Procesa la lectura cruda del joystick y la convierte en una
 * velocidad con:
 *  - Deadzone central
 *  - Mapeo lineal
 *  - Curva exponencial configurable
 *
 * @param value     Lectura ADC cruda
 * @param realMin   value mínimo ADC
 * @param center    Punto neutro calibrado
 * @param realMax   value máximo ADC
 * @return          Velocidad final (-MAX_SPEED a +MAX_SPEED)
 */

int processJoystick(int value, int realMin, int center, int realMax)
{

  value = constrain(value, realMin, realMax);

  if (abs(value - center) < DEADZONE)
    return 0;

  int lineal = 0;

  if (value < center)
  {
    lineal = map(value, realMin, center - DEADZONE, -MAX_SPEED, 0);
  }
  else
  {
    lineal = map(value, center + DEADZONE, realMax, 0, MAX_SPEED);
  }

  lineal = constrain(lineal, -MAX_SPEED, MAX_SPEED);

  float normalized = (float)lineal / MAX_SPEED;

  float curvado = pow(abs(normalized), EXPONENT);
  if (normalized < 0)
    curvado = -curvado;

  return (int)(curvado * MAX_SPEED);
}

/**
 * Lee los valuees de joystick y potenciómetro, procesa el joystick
 * y envía un paquete UDP binario al robot.
 *
 * El envío se hace directamente como bytes para minimizar overhead y latencia.
 * El paquete tiene un tamaño fijo de 5 bytes, lo que facilita su recepción y procesamiento en el robot.
 *
 */

void sendData()
{
  // LECTURA Y PROCESAMIENTO JOYSTICK
  int rawX = analogRead(JOY_X_PIN);
  int rawY = analogRead(JOY_Y_PIN);

  int vx = processJoystick(rawX, minX_Real, centerX, maxX_Real);
  int vy = processJoystick(rawY, minY_Real, centerY, maxY_Real);

  // LECTURA POTENCIÓMETRO
  int rawPot = analogRead(POT_PIN);
  int potValue = map(rawPot, 0, 4095, 0, 255); // Mapear a rango 0-255

  // CREAR PAQUETE BINARIO
  DataPacket packet;
  packet.joyX = (int16_t)vx;
  packet.joyY = (int16_t)vy;
  packet.pot = (uint8_t)potValue;

  // ENVÍO UDP BINARIO
  // Enviar directamente la estructura como bytes para evitar overhead de serialización
  udp.beginPacket(udpAddress, udpPort);
  udp.write((uint8_t *)&packet, sizeof(packet));
  udp.endPacket();

  // Debug opcional
  Serial.printf("Enviado -> VX:%d VY:%d POT:%d\n", vx, vy, potValue);
}

/**
 * Verifica la conexión WiFi y reconecta si es necesario.
 * Esto es crucial para mantener la comunicación estable con el robot,
 * especialmente en entornos con interferencias o cuando el dispositivo se mueve.
 */

void verifyConnection()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi perdido, reconectando...");
    WiFi.disconnect();
    WiFi.reconnect();
    delay(500);
  }
}

// SETUP
void setup()
{
  Serial.begin(115200);

  // Configuración ADC ESP32
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // -------- CALIBRACIÓN AUTOMÁTICA --------
  Serial.println("Calibrando centro...");
  delay(500);
  centerX = analogRead(JOY_X_PIN);
  centerY = analogRead(JOY_Y_PIN);

  Serial.print("centro X: ");
  Serial.println(centerX);
  Serial.print("centro Y: ");
  Serial.println(centerY);

  // -------- CONEXIÓN WIFI --------
  Serial.print("Conectando al Robot AP");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConectado!");
  Serial.print("IP local: ");
  Serial.println(WiFi.localIP());
  udp.begin(4211);
}

// LOOP PRINCIPAL
void loop()
{
  verifyConnection();

  // Enviar datos cada 20 ms (50 Hz)
  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL)
  {
    lastSend = now;
    sendData();
  }
}