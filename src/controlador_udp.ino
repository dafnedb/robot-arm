#include <WiFi.h>
#include <WiFiUdp.h>

// ===== CONFIGURACIÓN CONEXIÓN =====
const char* ssid = "MTDS_ROBOT_AP";
const char* password = "robotseguro";
const char* udpAddress = "192.168.4.1";
const int udpPort = 4210;

// ===== PINES JOYSTICK =====
const int JOY_X_PIN = 34;
const int JOY_Y_PIN = 35;

// ===== OBJETOS =====
WiFiUDP udp;

// ===== PARÁMETROS DE CONTROL =====
const int MAX_SPEED = 200;
const int SEND_INTERVAL = 20; // 50Hz

// --- CALIBRACIÓN FINA DEL JOYSTICK ---
const int minX_Real = 0;    
const int maxX_Real = 4095; 
const int minY_Real = 0;
const int maxY_Real = 4095;
const int DEADZONE = 30; 

// Curva de sensibilidad (1.0 = Lineal, 2.0 = Suave, 3.0 = Muy suave)
const float EXPONENTE = 2.0;

// Variables dinámicas para el centro (auto-calibración)
int centroX = 2048;
int centroY = 2048;
// Para evitar envíos demasiado frecuentes
unsigned long lastSend = 0; 

void setup() {
  Serial.begin(115200);

  // --- CONFIGURACIÓN ADC ESP32 ---
  // Resolución 0-4095
  analogReadResolution(12);
  // Permite leer hasta 3.3V
  analogSetAttenuation(ADC_11db);

  // --- CALIBRACIÓN AUTOMÁTICA DEL CENTRO ---
  // Se asume que el joystick está quieto al encender
  Serial.println("Calibrando centro...");
  delay(500);
  centroX = analogRead(JOY_X_PIN);
  centroY = analogRead(JOY_Y_PIN);

  // Mostrar valores calibrados
  Serial.print("Centro X detectado: "); Serial.println(centroX);
  Serial.print("Centro Y detectado: "); Serial.println(centroY);

  // --- CONEXIÓN WIFI ---
  Serial.print("Conectando al Robot AP");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado al Robot!");
  Serial.print("IP Asignada: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Re-conectar si se pierde el WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi perdido, reconectando...");
    WiFi.disconnect();
    WiFi.reconnect();
    delay(500);
    return;
  }

  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;
    
    // Leer valores crudos
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);
    
    // Procesar con la nueva función
    // Pasamos los límites específicos de cada eje
    int vx = procesarJoystick(rawX, minX_Real, centroX, maxX_Real);
    int vy = procesarJoystick(rawY, minY_Real, centroY, maxY_Real);

    // Preparar mensaje: vx,vy,timestamp
    char msg[64];
    snprintf(msg, sizeof(msg), "%d,%d,%lu", vx, vy, now);
    Serial.println(msg);
    
    // Hacer el envío UDP
    udp.beginPacket(udpAddress, udpPort);
    udp.print(msg);
    udp.endPacket();
  }
}

/*
 * Función para convertir lectura a velocidad
 * Incluye: Mapeo dividido (Split Mapping) y Curva Exponencial
*/
int procesarJoystick(int valor, int minReal, int centro, int maxReal) {
  // Limitar entrada a rangos físicos conocidos
  valor = constrain(valor, minReal, maxReal);

  // Zona Muerta
  if (abs(valor - centro) < DEADZONE) return 0;

  // Mapeo Lineal Inicial (Split Mapping)
  // Asegura que ambos lados tengan el mismo recorrido aunque el centro no sea perfecto
  int lineal = 0;
  if (valor < centro) {
    lineal = map(valor, minReal, centro - DEADZONE, -MAX_SPEED, 0);
  } else {
    lineal = map(valor, centro + DEADZONE, maxReal, 0, MAX_SPEED);
  }

  // Asegurar límites
  lineal = constrain(lineal, -MAX_SPEED, MAX_SPEED);

  // Aplicar Curva Exponencial (Suavizado)
  // Convertir a rango -1.0 a 1.0 para aplicar la curva
  float normalizado = (float)lineal / MAX_SPEED;

  // Elevamos a la potencia (manteniendo el signo)
  float curvado = pow(abs(normalizado), EXPONENTE);
  if (normalizado < 0) curvado = -curvado;

  // Retornar a escala de velocidad
  return (int)(curvado * MAX_SPEED);
}