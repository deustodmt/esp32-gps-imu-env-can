#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "M5UnitENV.h"
#include "MPU6886.h"

// Configuración GPS
#define GPS_RX_PIN 18
#define GPS_TX_PIN 17
#define GPS_BAUDRATE 9600
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;  // Objeto para procesar datos GPS

// Configuración I2C
#define I2C_SDA 8
#define I2C_SCL 9

// Objetos sensores
SHT3X sht3x;
QMP6988 qmp;
MPU6886 imu;

void setupSensors() {
  if (!qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, I2C_SDA, I2C_SCL, 400000U)) {
    Serial.println("Couldn't find QMP6988");
    while (1) delay(1);
  }

  if (!sht3x.begin(&Wire, SHT3X_I2C_ADDR, I2C_SDA, I2C_SCL, 400000U)) {
    Serial.println("Couldn't find SHT3X");
    while (1) delay(1);
  }
}

void printEnvSensorData() {
  if (sht3x.update()) {
    Serial.println("\n-----SHT3X-----");
    Serial.print("Temperature: ");
    Serial.print(sht3x.cTemp);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(sht3x.humidity);
    Serial.println("% rH");
  }

  if (qmp.update()) {
    Serial.println("\n-----QMP6988-----");
    Serial.print("Temperature: ");
    Serial.print(qmp.cTemp);
    Serial.println(" °C");
    Serial.print("Pressure: ");
    Serial.print(qmp.pressure);
    Serial.println(" Pa");
  }
}

void printGPSData() {
  Serial.println("\n-----GPS-----");
  
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print("° ");
    Serial.print(gps.location.rawLat().negative ? "S" : "N");
    
    Serial.print("\nLongitude: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print("° ");
    Serial.print(gps.location.rawLng().negative ? "W" : "E");
    
    Serial.print("\nSatellites: ");
    Serial.println(gps.satellites.value());
    
    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" m");
    
    Serial.print("Speed: ");
    Serial.print(gps.speed.kmph());
    Serial.println(" km/h");
    
    Serial.print("Date: ");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.println(gps.date.year());
    
    Serial.print("Time: ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
  } else {
    Serial.println("Waiting for GPS fix...");
    Serial.print("Satellites in view: ");
    Serial.println(gps.satellites.value());
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Inicializar GPS
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS inicializado");

  // Inicializar sensores I2C
  setupSensors();
}

void loop() {
  // Leer datos del GPS
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // Se procesó un nuevo dato GPS
      printGPSData();
    }
  }

  // Leer y mostrar datos de sensores ambientales
  printEnvSensorData();

  // Si no hay datos GPS por mucho tiempo, mostrar advertencia
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No se detectan datos GPS. Verificar conexiones.");
    while(true); // Detener ejecución
  }

  delay(1000); // Esperar 1 segundo entre lecturas
}