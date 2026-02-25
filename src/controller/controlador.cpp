/**
 * ==============================================================
 *  Remote Control for MTDs Robot - ESP32 Joystick Controller
 * ==============================================================
 *  Dispositive: ESP32
 * Description:
 *  This code implements a remote control system for the MTDs robot using an ESP32 microcontroller.
 * It reads two analog joysticks and a potentiometer to control the speed and direction of a surgery simulation robot.
 * The processed data is sent via UDP to the robot at a frequency of 50 Hz.
 * ==============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Joystick.h"

// CONNECTION SETTINGS
const char *ssid = "MTDS_ROBOT_AP";
const char *password = "robotseguro";
const char *udpAddress = "192.168.4.1";
const uint16_t udpPort = 4210;
// POTENTIOMETER PIN
const int POT_PIN = 32;
// JOYSTICKS
Joystick upperJoystick(34, 35);
Joystick lowerJoystick(36, 39);
WiFiUDP udp;
// Timing
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 20; // 20 ms → 50 Hz

// =======================================================
// DATA PACKET STRUCTURE
// =======================================================

/**
 * Data packet definition to be sent to the robot. The struct is packed to ensure no padding bytes are added, guaranteeing a fixed size of 5 bytes.
 *
 * The struct uses __attribute__((packed)) to enforce this guarantee.
 */

#pragma pack(push, 1)
typedef struct __attribute__((packed))
{
  int16_t joy1X;
  int16_t joy1Y;
  int16_t joy2X;
  int16_t joy2Y;
  uint8_t pot; // potentiometer value (0-180)
} DataPacket;
#pragma pack(pop)
// Static assertion to ensure the struct size is exactly 9 bytes (4 int16_t + 1 uint8_t)
static_assert(sizeof(DataPacket) == 9, "DataPacket struct size must be exactly 9 bytes");

// =======================================================
// FUNCTIONS
// =======================================================

/**
 *  Sends the processed joystick and potentiometer data via UDP.
 *  The data is sent in a binary format to minimize overhead and latency.
 *  The packet size is fixed at 9 bytes (4 int16_t + 1 uint8_t).
 *
 */

void sendData()
{
  // Joystick raw readings
  JoyPosition lowerJoystickPos = lowerJoystick.readPosition();
  JoyPosition upperJoystickPos = upperJoystick.readPosition();

  // Potentiometer reading
  int rawPot = analogRead(POT_PIN);
  int potValue = map(rawPot, 0, 4095, 0, 180); // Map to 0-180 range

  // Create data packet
  DataPacket packet;
  packet.joy1X = (int16_t)upperJoystickPos.x;
  packet.joy1Y = (int16_t)upperJoystickPos.y;
  packet.joy2X = (int16_t)lowerJoystickPos.x;
  packet.joy2Y = (int16_t)lowerJoystickPos.y;
  packet.pot = (uint8_t)potValue;

  // UPD transmission
  udp.beginPacket(udpAddress, udpPort);
  udp.write((uint8_t *)&packet, sizeof(packet));
  udp.endPacket();

  // Debugging output
  Serial.printf("Enviado -> vx1:%d vy1:%d vx2:%d vy2:%d POT:%d\n", upperJoystickPos.x, upperJoystickPos.y, lowerJoystickPos.x, lowerJoystickPos.y, potValue);
}

/**
 * Verify WiFi connection and attempt reconnection if lost. This function checks the connection status and, if disconnected, it will try to reconnect to the WiFi network. It also includes a small delay to avoid rapid reconnection attempts.
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

  // Configure ADC for potentiometer reading
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("Calibrando centro, no muevas los joysticks...");
  delay(500);
  // Calibrate joysticks to set the center position
  upperJoystick.calibrate();
  lowerJoystick.calibrate();
  Serial.println("Calibración completa!");

  // Wifi connection
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

// MAIN LOOP
void loop()
{
  verifyConnection();

  // Send data at the defined interval
  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL)
  {
    lastSend = now;
    sendData();
  }
}