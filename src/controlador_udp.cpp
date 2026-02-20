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

// PARÁMETROS DE CONTROL
const int MAX_SPEED = 200;
const int SEND_INTERVAL = 20; // 50 Hz

const int minX_Real = 0;
const int maxX_Real = 4095;
const int minY_Real = 0;
const int maxY_Real = 4095;
const int DEADZONE = 30;
const float EXPONENTE = 2.0;

int centroX = 2048;
int centroY = 2048;

unsigned long lastSend = 0;

// =======================================================
// DEFINICIÓN DEL PAQUETE BINARIO (8 BYTES EXACTOS)
// =======================================================

#pragma pack(push, 1)
typedef struct __attribute__((packed))
{
  int16_t joyX; // Cambiado a 16 bits con signo
  int16_t joyY; // Cambiado a 16 bits con signo
  uint8_t pot;  // El pot de 0-255 está perfecto en 8 bits sin signo
} DataPacket;
#pragma pack(pop)

// ACTUALIZAR EL CONTROL DE TAMAÑO
static_assert(sizeof(DataPacket) == 5, "El struct no mide 5 bytes");
// =======================================================
// PROCESAMIENTO JOYSTICK
// =======================================================

int procesarJoystick(int valor, int minReal, int centro, int maxReal)
{

  valor = constrain(valor, minReal, maxReal);

  if (abs(valor - centro) < DEADZONE)
    return 0;

  int lineal = 0;

  if (valor < centro)
  {
    lineal = map(valor, minReal, centro - DEADZONE, -MAX_SPEED, 0);
  }
  else
  {
    lineal = map(valor, centro + DEADZONE, maxReal, 0, MAX_SPEED);
  }

  lineal = constrain(lineal, -MAX_SPEED, MAX_SPEED);

  float normalizado = (float)lineal / MAX_SPEED;

  float curvado = pow(abs(normalizado), EXPONENTE);
  if (normalizado < 0)
    curvado = -curvado;

  return (int)(curvado * MAX_SPEED);
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
  centroX = analogRead(JOY_X_PIN);
  centroY = analogRead(JOY_Y_PIN);

  Serial.print("Centro X: ");
  Serial.println(centroX);
  Serial.print("Centro Y: ");
  Serial.println(centroY);

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

  // Reconexión automática
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi perdido, reconectando...");
    WiFi.disconnect();
    WiFi.reconnect();
    delay(500);
    return;
  }

  unsigned long now = millis();
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000)
  {
    Serial.print("Estado WiFi: ");
    Serial.println(WiFi.status());
    lastPrint = millis();
  }

  if (now - lastSend >= SEND_INTERVAL)
  {

    lastSend = now;

    // LECTURA Y PROCESAMIENTO JOYSTICK
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);

    int vx = procesarJoystick(rawX, minX_Real, centroX, maxX_Real);
    int vy = procesarJoystick(rawY, minY_Real, centroY, maxY_Real);

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
}