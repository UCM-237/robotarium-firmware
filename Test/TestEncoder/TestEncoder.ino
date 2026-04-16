
#include <math.h>
#include <Arduino.h>

#define ENCODER_CUADRATURA

#ifdef ENCODER_CUADRATURA
// The Arduino MKR WiFi 1010 supports external interrupts on almost all digital and analog pins, specifically 0, 1, 4, 5, 6, 7, 8, A1 (16), and A2 (17)

const int channelPinA_R = 0;
const int channelPinB_R = 1;

const int channelPinA_L = 5;
const int channelPinB_L = 4;

// ==================================================================================
// PROYECTO: Robotarium - Control de Encoders de Alta Resolución
// OBJETIVO: Lectura mediante máquina de estados (Modo 4x)
// ==================================================================================

volatile long countsL = 0;
volatile long countsR = 0;

// Variables para almacenar el estado anterior
volatile byte lastStateL = 0;
volatile byte lastStateR = 0;

void isrL() {
  // 1. Leer ambos pines y formar un número de 2 bits (binario)
  byte currentState = (digitalRead(5) << 1) | digitalRead(4);
  
  // 2. Comparar con el estado anterior para saber dirección
  // Esta lógica es un "resumen" de la tabla de verdad de cuadratura
  if (lastStateL != currentState) {
    if ((lastStateL == 0 && currentState == 1) || 
        (lastStateL == 1 && currentState == 3) || 
        (lastStateL == 3 && currentState == 2) || 
        (lastStateL == 2 && currentState == 0)) {
      countsL++;
    } else {
      countsL--;
    }
    lastStateL = currentState;  
  }
}

void isrR() {
  byte currentState = (digitalRead(0) << 1) | digitalRead(1);
  if (lastStateR != currentState) {
    if ((lastStateR == 0 && currentState == 1) || 
        (lastStateR == 1 && currentState == 3) || 
        (lastStateR == 3 && currentState == 2) || 
        (lastStateR == 2 && currentState == 0)) {
      countsR++;
    } else {
      countsR--;
    }
    lastStateR = currentState;
  }
}
void setup() {
  Serial.begin(115200);

  pinMode(channelPinA_L, INPUT_PULLUP); // Canal A Izq
  pinMode(channelPinB_L , INPUT_PULLUP); // Canal B Izq
  pinMode(channelPinA_R , INPUT_PULLUP); // Canal A Der
  pinMode(channelPinB_R , INPUT_PULLUP); // Canal B Der

  // Interrumpimos en AMBOS canales para no perder ni un solo paso
  attachInterrupt(digitalPinToInterrupt(5), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(0), isrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(1), isrR, CHANGE);
}


void loop() {
  static unsigned long t = 0;
  if (millis() - t > 100) {
    Serial.print("L: "); Serial.print(countsL);
    Serial.print(" | R: "); Serial.println(countsR);
    t = millis();
  }
}
#else
// Definiciones basadas en tus archivos
const int pinEncoderIzquierdo = 3; // Ajusta según tu robot.h
const int pinEncoderDerecho = 2;   // Ajusta según tu robot.h

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
  if (ahora - ultimoMicrosDerecho > 50000) {
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

#endif
