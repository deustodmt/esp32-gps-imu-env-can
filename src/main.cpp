#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "M5UnitENV.h"
#include "MPU6886.h"
#include <CAN_config.h>
#include <ESP32CAN.h>

// Configuración CAN
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
CAN_device_t CAN_cfg; // Configuración CAN

// Configuración GPS
#define GPS_RX_PIN 18
#define GPS_TX_PIN 17
#define GPS_BAUDRATE 9600
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// Configuración I2C
#define I2C_SDA 8
#define I2C_SCL 9

// Objetos sensores
SHT3X sht3x;
QMP6988 qmp;
MPU6886 imu;

// Variables para calibración IMU
float accelBias[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0};
const uint16_t calibrationSamples = 500;

// Estructura para datos CAN
typedef struct {
  float latitude;
  float longitude;
  float altitude;
  float speed;
  float temperature;
  float humidity;
  float pressure;
  float accel[3];
  float gyro[3];
  uint8_t satellites;
} SensorData_t;

void setupCAN() {
  CAN_cfg.speed = CAN_SPEED_125KBPS;
  CAN_cfg.tx_pin_id = (gpio_num_t)CAN_TX_PIN;
  CAN_cfg.rx_pin_id = (gpio_num_t)CAN_RX_PIN;
  CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
  
  // Inicializar CAN
  if (ESP32Can.CANInit() == 0)
    Serial.println("CAN inicializado");
  else
    Serial.println("ERROR al inicializar CAN");
}

void sendCANData(const SensorData_t& data) {
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x123; // ID del mensaje (puedes cambiarlo)
  tx_frame.FIR.B.DLC = 8; // Longitud de datos (máximo 8 bytes por frame)
  
  // Enviamos los datos en múltiples frames si es necesario
  // Frame 1: Posición (latitud y longitud)
  memcpy(tx_frame.data.u8, &data.latitude, 4);
  memcpy(tx_frame.data.u8 + 4, &data.longitude, 4);
  Serial.println("hasta aqui llega");
  ESP32Can.CANWriteFrame(&tx_frame);
  Serial.println("aqui no");
  delay(10);
  
  // Frame 2: Altitud y velocidad
  memcpy(tx_frame.data.u8, &data.altitude, 4);
  memcpy(tx_frame.data.u8 + 4, &data.speed, 4);
  ESP32Can.CANWriteFrame(&tx_frame);
  delay(10);
  
  // Frame 3: Datos ambientales
  memcpy(tx_frame.data.u8, &data.temperature, 4);
  memcpy(tx_frame.data.u8 + 4, &data.humidity, 4);
  ESP32Can.CANWriteFrame(&tx_frame);
  delay(10);
  
  // Frame 4: Presión y satélites
  memcpy(tx_frame.data.u8, &data.pressure, 4);
  tx_frame.data.u8[4] = data.satellites;
  ESP32Can.CANWriteFrame(&tx_frame);
  delay(10);
  
  // Frame 5: Acelerómetro (X e Y)
  memcpy(tx_frame.data.u8, &data.accel[0], 4);
  memcpy(tx_frame.data.u8 + 4, &data.accel[1], 4);
  ESP32Can.CANWriteFrame(&tx_frame);
  delay(10);
  
  // Frame 6: Acelerómetro (Z) y Giroscopio (X)
  memcpy(tx_frame.data.u8, &data.accel[2], 4);
  memcpy(tx_frame.data.u8 + 4, &data.gyro[0], 4);
  ESP32Can.CANWriteFrame(&tx_frame);
  delay(10);
  
  // Frame 7: Giroscopio (Y y Z)
  memcpy(tx_frame.data.u8, &data.gyro[1], 4);
  memcpy(tx_frame.data.u8 + 4, &data.gyro[2], 4);
  ESP32Can.CANWriteFrame(&tx_frame);
  
  Serial.println("Datos enviados por CAN");
}

void calibrateIMU() {
  Serial.println("Calibrando IMU (mantener estable)...");
  
  float accelSum[3] = {0, 0, 0};
  float gyroSum[3] = {0, 0, 0};
  
  for(uint16_t i=0; i<calibrationSamples; i++) {
    float acc[3], gyro[3];
    imu.getAccelData(&acc[0], &acc[1], &acc[2]);
    imu.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
    
    for(int j=0; j<3; j++) {
      accelSum[j] += acc[j];
      gyroSum[j] += gyro[j];
    }
    delay(10);
  }
  
  // Calcular bias
  for(int i=0; i<3; i++) {
    accelBias[i] = accelSum[i]/calibrationSamples;
    if(i==2) accelBias[i] -= 1.0; // Compensar gravedad en Z
    gyroBias[i] = gyroSum[i]/calibrationSamples;
  }
  
  Serial.println("Calibración IMU completada");
}

void setupSensors() {
  // Inicializar QMP6988
  if (!qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, I2C_SDA, I2C_SCL, 400000U)) {
    Serial.println("Couldn't find QMP6988");
    while (1) delay(1);
  }

  // Inicializar SHT3X
  if (!sht3x.begin(&Wire, SHT3X_I2C_ADDR, I2C_SDA, I2C_SCL, 400000U)) {
    Serial.println("Couldn't find SHT3X");
    while (1) delay(1);
  }

  // Inicializar MPU6886
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  if (imu.Init()) {
    Serial.println("Couldn't find MPU6886");
    while (1) delay(1);
  }
  
  // Configurar rangos
  imu.setAccelFsr(MPU6886::AFS_4G);
  imu.setGyroFsr(MPU6886::GFS_500DPS);
  
  // Calibrar IMU
  calibrateIMU();
}

void readSensors(SensorData_t& data) {
  // Leer datos GPS
  data.latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
  data.longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
  data.altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  data.speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  data.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
  
  // Leer datos ambientales
  if (sht3x.update()) {
    data.temperature = sht3x.cTemp;
    data.humidity = sht3x.humidity;
  }
  
  if (qmp.update()) {
    data.pressure = qmp.pressure;
  }
  
  // Leer datos IMU
  float acc[3], gyro[3];
  imu.getAccelData(&acc[0], &acc[1], &acc[2]);
  imu.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
  
  // Aplicar calibración
  for(int i=0; i<3; i++) {
    data.accel[i] = acc[i] - accelBias[i];
    data.gyro[i] = gyro[i] - gyroBias[i];
  }
}

void printIMUData() {
  float acc[3], gyro[3], temp;
  
  // Leer datos con corrección de bias
  imu.getAccelData(&acc[0], &acc[1], &acc[2]);
  imu.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
  imu.getTempData(&temp);

  // Aplicar calibración
  for(int i=0; i<3; i++) {
    acc[i] -= accelBias[i];
    gyro[i] -= gyroBias[i];
  }

  // Mostrar datos
  Serial.println("\n-----MPU6886-----");
  Serial.print("Acelerómetro (g): X=");
  Serial.print(acc[0], 2);
  Serial.print(" Y=");
  Serial.print(acc[1], 2);
  Serial.print(" Z=");
  Serial.println(acc[2], 2);
  
  Serial.print("Giroscopio (°/s): X=");
  Serial.print(gyro[0], 1);
  Serial.print(" Y=");
  Serial.print(gyro[1], 1);
  Serial.print(" Z=");
  Serial.println(gyro[2], 1);
  
  Serial.print("Temperatura IMU: ");
  Serial.print(temp, 1);
  Serial.println(" °C");
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

  // Inicializar CAN
  setupCAN();

  // Inicializar GPS
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS inicializado");

  // Inicializar todos los sensores
  setupSensors();
}

void loop() {
  static uint32_t lastEnvUpdate = 0;
  static uint32_t lastIMUUpdate = 0;
  static uint32_t lastCANUpdate = 0;
  
  // Procesar datos GPS continuamente
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Actualizar sensores ambientales cada segundo
  if (millis() - lastEnvUpdate >= 1000) {
    printGPSData();
    printEnvSensorData();
    lastEnvUpdate = millis();
  }

  // Actualizar IMU más frecuentemente (10Hz)
  if (millis() - lastIMUUpdate >= 100) {
    printIMUData();
    lastIMUUpdate = millis();
  }

  // Enviar datos por CAN cada 2 segundos
  if (millis() - lastCANUpdate >= 2000) {
    SensorData_t sensorData;
    readSensors(sensorData);
    sendCANData(sensorData);
    lastCANUpdate = millis();
  }

  // Verificar fallo GPS
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No se detectan datos GPS. Verificar conexiones.");
  }
}