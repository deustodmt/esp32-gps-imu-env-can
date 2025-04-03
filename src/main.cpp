#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Crear el objeto GPS
TinyGPSPlus gps;

// Configurar el puerto serial del ESP32 para la comunicación con el GPS
HardwareSerial mySerial(1);

void setup() {
  // Inicializar la comunicación serial con el monitor
  Serial.begin(9600);  // Configura el baud rate aquí

  Serial.println("Empezando conexion serial con el GPS...");
  // Inicializar el puerto serial para el GPS en los pines 17 y 18
  mySerial.begin(9600, SERIAL_8N1, 17, 18);
  
  Serial.println("Iniciando GPS...");
}

void loop() {
  // Leer los datos del GPS y pasarlos al objeto TinyGPS++
  while (mySerial.available() > 0) {
    gps.encode(mySerial.read());
    Serial.println("Leyendo datos...");
  }
  
  // Comprobar si hay nuevos datos de GPS disponibles
  if (gps.location.isUpdated()) {
    Serial.print("Latitud= "); 
    Serial.print(gps.location.lat(), 6); 
    Serial.print(" Longitud= "); 
    Serial.println(gps.location.lng(), 6); 
  }
  else {
    Serial.println("No data");
  }
  
  delay(1000);  // Esperar un segundo
}
