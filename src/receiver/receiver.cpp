#include <WiFi.h>
#include <WiFiUdp.h>

// =======================================================
// NET SETTINGS
// =======================================================
const char *ssid = "MTDS_ROBOT_AP";
const char *password = "robotseguro";
const uint16_t localPort = 4210;

// =======================================================
// GENERAL SETTINGS
// =======================================================
constexpr uint8_t MAX_CLIENTS = 1;    // Max number of clients that can connect to the AP
constexpr uint32_t TIMEOUT_MS = 1000; // Timeout for UDP packet reception (1 second)

WiFiUDP udp;
HardwareSerial MegaSerial(2); // UART2 (RX=16, TX=17)

unsigned long lastPacketTime = 0;

// =======================================================
// DATA PACKET STRUCTURE
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

// =======================================================
// SETUP
// =======================================================
void setup()
{
  Serial.begin(115200);
  MegaSerial.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("Iniciando Access Point...");
  // Initiate WiFi in AP mode
  if (!WiFi.softAP(ssid, password, 6, false, MAX_CLIENTS))
  {
    Serial.println("Error iniciando Access Point");
    while (true)
      ; // Block execution if AP fails to start
  }
  Serial.print("IP del AP: ");
  Serial.println(WiFi.softAPIP());
  // Iniciar UDP
  if (!udp.begin(localPort))
  {
    Serial.println("Error iniciando UDP");
    while (true)
      ;
  }
  Serial.printf("Escuchando UDP en puerto %d\n", localPort);
}

// =======================================================
// LOOP PRINCIPAL
// =======================================================
void loop()
{
  int packetSize = udp.parsePacket();
  // Only process if a packet is received and it's not too old
  if (packetSize == sizeof(DataPacket))
  {
    DataPacket packet;

    int len = udp.read((uint8_t *)&packet, sizeof(packet));

    if (len != sizeof(packet))
    {
      Serial.println("Error leyendo paquete");
      return;
    }

    // ===============================================
    // Clean data extraction
    // ===============================================
    int vx1 = packet.upperJoyX;
    int vy1 = packet.upperJoyY;
    int vx2 = packet.lowerJoyX;
    int vy2 = packet.lowerJoyY;
    int pot = packet.pot;

    // Debugging output
    Serial.printf("VX1:%d VY1:%d VX2:%d VY2:%d POT:%d\n", vx1, vy1, vx2, vy2, pot);

    // ===============================================
    // SEND TO MEGA VIA UART
    // ===============================================
    MegaSerial.printf("<%d,%d,%d,%d,%d>\n", vx1, vy1, vx2, vy2, pot);
  }
}