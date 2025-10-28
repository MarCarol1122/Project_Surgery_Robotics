#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250.h"
#include <Wire.h> // Needed for I2C to read IMU
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <IMU_RoboticsUB.h>   // Nom de la llibreria custom

  
// Device ID
const char *deviceId = "G2_Gri";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// Vibration motor settings
const int vibrationPin = 23; // Pin for the vibration motor

// Botons
const int PIN_S1 = 14;
const int PIN_S2 = 27;
int s1Status = HIGH;
int s2Status = HIGH;

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 23); // IP of receiver ESP32
IPAddress receiverComputerIP(192, 168, 1, 25); // IP of PC
const int udpPort = 12345;
WiFiUDP udp;

// IMU object
IMU imu;

// Orientation data
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void updateOrientation() {
  // Llegeix FIFO del DMP i actualitza càlculs interns
  imu.ReadSensor();
  // Obté els angles (roll, pitch, yaw) via GetRPW()
  float* rpw = imu.GetRPW();
  Gri_roll  = rpw[0];
  Gri_pitch = rpw[1];
  Gri_yaw   = rpw[2];
  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);
}

void sendOrientationUDP() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["roll"] = Gri_roll;
  doc["pitch"] = Gri_pitch;
  doc["yaw"] = Gri_yaw;
  doc["s1"] = s1Status;
  doc["s2"] = s2Status;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);

  // Send to ESP32 Servos
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Send to Computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  // Inicialitza IMU (amb DMP)
  imu.Install();

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);

  // --- Configurar PWM para el motor de vibración ---
  // Canal 0, frecuencia de 5 kHz, resolución de 8 bits
  ledcSetup(0, 5000, 8);
  // Asociar el pin del motor de vibración al canal 0
  ledcAttachPin(vibrationPin, 0);
}

void receiveTorquesUDP() {
  int packetSize = udp.parsePacket();  // Ver si llegó un paquete UDP
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0; // Final de cadena
    }

    Serial.print("Received UDP packet: ");
    Serial.println(packetBuffer);

    // --- Parsear los valores de torque recibidos ---
    float Torque_roll1 = 0, Torque_pitch = 0, Torque_yaw = 0;
    sscanf(packetBuffer, "%f %f %f", &Torque_roll1, &Torque_pitch, &Torque_yaw);

    // --- Controlar el motor de vibración según el torque total ---
    float totalTorque = Torque_roll1 + Torque_pitch + Torque_yaw;

    // Convertir torque a valor PWM (0–255)
    int vibrationValue = constrain(totalTorque * 2.5, 0, 255);  // Ajusta el factor 2.5 si la vibración es muy débil o fuerte
    ledcWrite(0, vibrationValue);  // Aplica el PWM al canal 0 (motor de vibración)

    Serial.print("Vibration motor value: ");
    Serial.println(vibrationValue);
  }
}


void loop() {
  updateOrientation();
  sendOrientationUDP();
  receiveTorquesUDP();
  delay(10);
}
