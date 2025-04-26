// PRACTICA 1

/*
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

*/

