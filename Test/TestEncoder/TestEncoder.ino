
#include <math.h>
#include <Arduino.h>

// Definiciones basadas en tus archivos
const int pinEncoderIzquierdo = 1; // Ajusta según tu robot.h
const int pinEncoderDerecho = 0;   // Ajusta según tu robot.h

volatile unsigned long pulsosIzquierdo = 0;
volatile unsigned long pulsosDerecho = 0;
volatile unsigned long ultimoMicrosIzquierdo = 0;
volatile unsigned long ultimoMicrosDerecho = 0;
volatile unsigned long deltaTimeIzquierdo = 0;
volatile unsigned long deltaTimeDerecho = 0;

// Umbral de debounce: 1000 microsegundos (1ms)
const unsigned long DEBOUNCE_TIME = 30000; 

void isrIzquierda() {
  unsigned long ahora = micros();
  if (ahora - ultimoMicrosIzquierdo > DEBOUNCE_TIME) {
    deltaTimeIzquierdo = ahora - ultimoMicrosIzquierdo;
    ultimoMicrosIzquierdo = ahora;
    pulsosIzquierdo++;
  }
}

void isrDerecha() {
  unsigned long ahora = micros();
  if (ahora - ultimoMicrosDerecho > DEBOUNCE_TIME) {
    deltaTimeDerecho = ahora - ultimoMicrosDerecho;
    ultimoMicrosDerecho = ahora;
    pulsosDerecho++;
  }
}

void setup() {
  Serial.begin(115200); // Sube la velocidad para no perder tiempo en el loop
  pinMode(pinEncoderIzquierdo, INPUT_PULLUP);
  pinMode(pinEncoderDerecho, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(pinEncoderIzquierdo), isrIzquierda, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinEncoderDerecho), isrDerecha, FALLING);
  
  Serial.println("--- Test de Encoders Iniciado ---");
}

void loop() {
  static unsigned long ultimaImpresion = 0;
  if (millis() - ultimaImpresion > 250) { // Imprime cada 250ms
    Serial.print("IZQ: "); Serial.print(pulsosIzquierdo);
    Serial.print(" | Δt IZQ: "); Serial.print(deltaTimeIzquierdo);
    Serial.print(" us | DER: "); Serial.print(pulsosDerecho);
    Serial.print(" | Δt DER: "); Serial.println(deltaTimeDerecho);
    ultimaImpresion = millis();
  }
}
