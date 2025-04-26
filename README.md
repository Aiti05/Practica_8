## **PRACTICA 8 : Buses de comunicación IV (uart)**



## ** Ejercicio practico 1 bucle de comunicacion uart2:**
**Codigo main.cpp:**
```
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("Sistema iniciado. Escribe algo:");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial2.write(c);
    Serial.print(">> Enviado a UART2: ");
    Serial.println(c);
  }

  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.write("<< Recibido de UART2: ");
    Serial.println(c);
  }
}

```
En el primer ejercicio práctico se implementa una comunicación en bucle utilizando el puerto UART2 del ESP32. En la función setup(), 
se inicializan los puertos Serial y Serial2, ambos configurados a 115200 baudios, permitiendo la transmisión entre el monitor serie y 
los pines físicos GPIO16 (RX) y GPIO17 (TX).

Durante la ejecución del programa en el loop(), se comprueba continuamente si existen datos disponibles en Serial. Si es así, se leen los 
caracteres y se reenvían por Serial2, mostrando además un mensaje en el monitor serie indicando el dato enviado. De forma similar, si llegan 
datos a Serial2, se reciben y se reenvían al monitor serie.

Este ejercicio permite entender de manera práctica cómo funciona una comunicación asíncrona básica entre dos puertos UART del ESP32, y sienta 
las bases para conectar dispositivos externos en proyectos más avanzados.

La salida que aparece en el monitor al escribir algun caracter es:
```
>> Enviado a UART2: k
<< Recibido de UART2: k
>> Enviado a UART2: m
<< Recibido de UART2: m
>> Enviado a UART2: p
<< Recibido de UART2: p
>> Enviado a UART2: i
<< Recibido de UART2: i
>> Enviado a UART2: i
<< Recibido de UART2: i
...
```

## ** Ejercicio practico 2 (optativo) modulo GPS:**
**Codigo main.cpp:**
```
#include <TinyGPS.h>
#include <HardwareSerial.h>

TinyGPS gps;
HardwareSerial SerialGPS(1); // Usamos UART1 (puedes cambiar los pines si es necesario)

void setup() {
  Serial.begin(115200); // Comunicación con PC
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // Comunicación con GPS, RX=16, TX=17
  Serial.println("Iniciando recepción de datos GPS...");
}

void loop() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // Intentamos recibir datos durante un segundo
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (SerialGPS.available()) {
      char c = SerialGPS.read();
      if (gps.encode(c)) {
        newData = true;
      }
    }
  }

  if (newData) {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }

  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
}

```
En este programa se utiliza la librería TinyGPS junto con un puerto UART adicional (SerialGPS) para establecer comunicación con un módulo 
GPS externo. En el setup() se inicializan tanto la comunicación con el PC como la comunicación con el módulo GPS a través de los pines 
GPIO16 y GPIO17 del ESP32. En el loop(), durante un intervalo de un segundo, el programa lee los datos que llegan por UART1. Cada carácter 
recibido se pasa al objeto gps que decodifica los mensajes NMEA. Si se recibe una secuencia válida, se extraen y muestran en el monitor 
serie la latitud, la longitud, el número de satélites y la precisión de la señal. También se imprimen estadísticas como el número de 
caracteres procesados, frases válidas y errores de checksum.
Este ejercicio permite visualizar en tiempo real la información geográfica recibida del GPS.

## **Ejercicio practico 3 (optativo) modulo GPRS // GSM:**
**Codigo main.cpp:**
```
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Wire.h>

#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial
#define SerialAT Serial1

const char apn[] = "internet"; // Cambiar por APN real
const char gprsUser[] = "";
const char gprsPass[] = "";
const char* broker = "test.mosquitto.org";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

void setup() {
  SerialMon.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, 26, 27); // RX=26, TX=27
  delay(3000);

  SerialMon.println("Iniciando modem...");
  modem.restart();
  
  SerialMon.println("Conectando a red móvil...");
  modem.gprsConnect(apn, gprsUser, gprsPass);

  if (modem.isGprsConnected()) {
    SerialMon.println("Conectado a GPRS");
  } else {
    SerialMon.println("Error de conexión GPRS");
  }

  mqtt.setServer(broker, 1883);
}

void loop() {
  if (!mqtt.connected()) {
    SerialMon.println("MQTT desconectado. Intentando reconectar...");
    if (mqtt.connect("ESP32Client")) {
      SerialMon.println("Conectado a MQTT broker");
      mqtt.subscribe("esp/test");
    } else {
      SerialMon.println("Fallo al conectar MQTT");
      delay(5000);
      return;
    }
  }

  long now = millis();
  static long lastSend = 0;
  if (now - lastSend > 10000) {
    lastSend = now;
    mqtt.publish("esp/test", "Hola desde ESP32 GSM");
    SerialMon.println("Mensaje enviado");
  }

  mqtt.loop();
}


```

Este programa establece una conexión a internet mediante un módulo GSM/GPRS como el SIM800L. Primero se inicializan los puertos serie para 
comunicación local (monitor serie) y comunicación con el modem GSM. A continuación, se reinicia el modem y se establece una conexión a la 
red móvil utilizando el APN correspondiente al proveedor de servicios. Una vez conectados, el programa configura un cliente MQTT para 
enviar y recibir mensajes a través de un broker público (en este caso test.mosquitto.org). En el bucle principal, si no existe conexión 
MQTT, el sistema intenta reconectarse automáticamente. Cada 10 segundos, si está conectado, el ESP32 publica un mensaje de prueba en el 
tema esp/test.
Este ejercicio demuestra cómo utilizar el ESP32 junto con un módulo GSM para enviar datos a servidores en internet, algo fundamental en 
proyectos IoT (Internet of Things).
