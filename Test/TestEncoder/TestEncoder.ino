
#include <math.h>
#include <Arduino.h>

#define ENCODER_CUADRATURA

#ifdef ENCODER_CUADRATURA
// The Arduino MKR WiFi 1010 supports external interrupts on almost all digital and analog pins, specifically 0, 1, 4, 5, 6, 7, 8, A1 (16), and A2 (17)

const int channelPinA_R = 0;
const int channelPinB_R = 1;

const int channelPinA_L = 4;
const int channelPinB_L = 5;

// Umbral de debounce: 1000 microsegundos (1ms)
const unsigned long DEBOUNCETIME = 30000; 
volatile unsigned long ultimoMicrosIzquierdo = 0;
volatile unsigned long ultimoMicrosDerecho = 0;
volatile unsigned long deltaTimeIzquierdo = 0;
volatile unsigned long deltaTimeDerecho = 0;

const int maxSteps = 7;
volatile int ISRCounter_R = 0;
volatile int ISRCounter_L = 0;

int counter_R = 0;
int counter_L = 0;

bool IsCW_R = true;
bool IsCW_L = true;

void setup()
{
  pinMode(channelPinA_R, INPUT_PULLUP);
  pinMode(channelPinB_R, INPUT_PULLUP);
  pinMode(channelPinA_L, INPUT_PULLUP);
  pinMode(channelPinB_L, INPUT_PULLUP);
  
  Serial.begin(9600);
  
  attachInterrupt(digitalPinToInterrupt(channelPinA_R), doEncodeA_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelPinB_R), doEncodeB_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelPinA_L), doEncodeA_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelPinB_L), doEncodeB_L, CHANGE);

}

void loop()
{
  static unsigned long ultimaImpresion = 0;
  if (millis() - ultimaImpresion > 250) { // Imprime cada 250ms
    Serial.print("IZQ: "); Serial.print(ISRCounter_L);
    Serial.print(" | Δt IZQ: "); Serial.print(deltaTimeIzquierdo);
    Serial.print(" us | DER: "); Serial.print(ISRCounter_R);
    Serial.print(" | Δt DER: "); Serial.println(deltaTimeDerecho);
    ultimaImpresion = millis();
  }
}

// Rueda izquierda

void doEncodeA_L()
{
  unsigned long ahora = micros();
  if (ahora -ultimoMicrosIzquierdo> DEBOUNCETIME)
  {
    if (digitalRead(channelPinA_L) == digitalRead(channelPinB_L))
    {
      IsCW_L = true;
      ISRCounter_L++;
    }
    else
    {
      IsCW_L = false;
      ISRCounter_L--;
    }
    deltaTimeIzquierdo = ahora - ultimoMicrosIzquierdo;
    ultimoMicrosIzquierdo = ahora;  
    }
}

void doEncodeB_L()
{
  unsigned long ahora = micros();
  if (ahora -ultimoMicrosIzquierdo> DEBOUNCETIME)
 {
    if (digitalRead(channelPinA_L) != digitalRead(channelPinB_L))
    {
      IsCW_L = true;
      ISRCounter_L++;
    }
    else
    {
      IsCW_L = false;
      ISRCounter_L--;
    }
    deltaTimeIzquierdo = ahora - ultimoMicrosIzquierdo;
    ultimoMicrosIzquierdo = ahora;  ;
  }
}

// Rueda derecha
void doEncodeA_R()
{
   unsigned long ahora = micros();
  if (ahora -ultimoMicrosDerecho> DEBOUNCETIME)
  {
    if (digitalRead(channelPinA_R) == digitalRead(channelPinB_R))
    {
      IsCW_R = true;
      if (ISRCounter_R + 1 <= maxSteps) ISRCounter_R++;
    }
    else
    {
      IsCW_R = false;
      if (ISRCounter_R - 1 > 0) ISRCounter_R--;
    }
    deltaTimeDerecho= ahora - ultimoMicrosDerecho;
    ultimoMicrosDerecho = ahora;  

  }
}

void doEncodeB_R()
{
  unsigned long ahora = micros();
  if (ahora -ultimoMicrosDerecho > DEBOUNCETIME)
  {
    if (digitalRead(channelPinA_R) != digitalRead(channelPinB_R))
    {
      IsCW_R = true;
      if (ISRCounter_R + 1 <= maxSteps) ISRCounter_R++;
    }
    else
    {
      IsCW_R = false;
      if (ISRCounter_R - 1 > 0) ISRCounter_R--;
    }
      deltaTimeDerecho= ahora - ultimoMicrosDerecho;
    ultimoMicrosDerecho = ahora;  

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
