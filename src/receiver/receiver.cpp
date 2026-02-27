#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>

HardwareSerial MegaSerial(2); // UART2 (RX=16, TX=17)

// =======================================================
// DATA PACKET STRUCTURE (9 bytes)
// =======================================================
#pragma pack(push, 1)
typedef struct __attribute__((packed))
{
  int16_t upperJoyX; // -200 a 200
  int16_t upperJoyY; // -200 a 200
  int16_t lowerJoyX; // -200 a 200
  int16_t lowerJoyY; // -200 a 200
  uint8_t pot;       // 0 a 255
} DataPacket;
#pragma pack(pop)
static_assert(sizeof(DataPacket) == 9, "Error Crítico: El struct no mide 9 bytes");

DataPacket packet;

// =======================================================
// CALLBACK: ESP-NOW data sent callback
// =======================================================
void OnDataRecv(const uint8_t * mac, const uint8_t * incomingData, int len) {
  if (len == sizeof(DataPacket)) {
    // Copy incoming data to our packet struct
    memcpy(&packet, incomingData, sizeof(packet));
    // Send the data to the Mega via UART
    MegaSerial.printf("<%d,%d,%d,%d,%d>\n", packet.upperJoyX, packet.upperJoyY, packet.lowerJoyX, packet.lowerJoyY, packet.pot);
    // Debug output
    Serial.printf("Recibido -> VX1:%d VY1:%d VX2:%d VY2:%d POT:%d\n", packet.upperJoyX, packet.upperJoyY, packet.lowerJoyX, packet.lowerJoyY, packet.pot);
  }
}

// =======================================================
// SETUP
// =======================================================
void setup()
{
  Serial.begin(115200);
  MegaSerial.begin(115200, SERIAL_8N1, 16, 17);
  // Initialize ESP-NOW. Set the ESP32 as a Wi-Fi station (client) to use ESP-NOW
  WiFi.mode(WIFI_STA);
  // Validate that ESP-NOW is initialized successfully
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP-NOW Iniciado. Esperando datos...");
}

// =======================================================
// MAIN LOOP
// =======================================================
void loop()
{
  // The main loop is empty because we are using a callback to receive data asynchronously. All processing happens in the OnDataRecv callback function.
  delay(1); // Small delay to prevent watchdog timer reset, if necessary
}