/*
 * ----------------------------------------------------------------------------
 * PROYECTO: Robotarium Hub (UCM)
 * ARCHIVO:  Robot.ino
 * RUTA:     /firmware/Robot.ino
 * REPO:     https://github.com/UCM-237/robotarium-hub
 * ----------------------------------------------------------------------------
*/


#include "common.h"
#include "controler.h"
#include "robot.h"
#ifdef ARDUINO_TYPE_MKR
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <ArduinoJson.h>
#endif


// Configuración de macros para depuración por el Monitor Serie
#define DEBUG_ENABLED  


#ifdef DEBUG_ENABLED
#define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)   
#define DEBUG_PRINTLN(...) 
#endif

// --- CONFIGURACIÓN DEL TEST ---
double VELOCIDAD_OBJETIVO = 0.01; // rad/s (ajusta según necesites)
bool TEST_RUEDA_DERECHA = true;   // true para derecha, false para izquierda
bool BACKWARDS= false; // true rueda hacia atras, false hacia adelante
// ------------------------------
// Uncomment only one
//#define  AJUSTEFF
//#define TESTFF
#define AJUSTEPID
//------------------------------
int pwm_output=0;
double w_objetivo=VLMIN;
robot miRobot;
controler PID_RuedaL, PID_RuedaR;


/// --- FILTROS Y ESTABILIZACIÓN ---
// Filtros de media móvil para suavizar las lecturas de velocidad de los encoders (ventana de 10 muestras)
MeanFilter<long> meanFilterRight(10);
MeanFilter<long> meanFilterLeft(10);
const double MAX_OPTIMAL_VEL=20; // Límite de seguridad en rad/s


void setup() {
  Serial.begin(9600);
  miRobot.pinSetup();
  miRobot.motorSetup();
  // Paso 1: Objetivo realista (aprox 2 vueltas por segundo)
  VELOCIDAD_OBJETIVO = 15.0; 
  PID_RuedaL.setSetPoint(VELOCIDAD_OBJETIVO);
  PID_RuedaR.setSetPoint(VELOCIDAD_OBJETIVO);
  
  /* Rueda izquierda
   *  A=18,75
   *  B=-106,25
   *  PWM_MIN=100 w=11 r/s
   *  PWM_MAX=255 w=19 r/s
   *  Rueda derecha
   *  A=18.75
   *  B=-68,75
   */
  PID_RuedaL.setFeedForwardParam(18.75,-68.75);
  PID_RuedaR.setFeedForwardParam(18.75,-106.25);
  // Paso 2: Solo proporcional (Kp). Ki y Kd a CERO.
  // Un Kp de 2.0 o 5.0 es un buen inicio para motores de bajo coste.
  PID_RuedaL.setControlerParam(15.0, 1.0,0.0);
  PID_RuedaR.setControlerParam(15.0, 1.0,0.0);
  // Configuración de interrupciones para encoders (necesario para calcular w real)
  attachInterrupt(digitalPinToInterrupt(miRobot.getPinLeftEncoder()), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(miRobot.getPinRightEncoder()), isrRight, RISING);
      // Inicializamos los tiempos para que la resta no dé valores extraños
    noInterrupts();
    timeAfterLeft = micros();
    timeAfterRight = micros();
    deltaTimeLeft = 0; 
    deltaTimeRight = 0;
    interrupts();
  
  // Opcional: Si quieres que el filtro empiece "limpio"
  for(int i=0; i<10; i++) {
    meanFilterLeft.AddValue(0.0);
    meanFilterRight.AddValue(0.0);
  }


  Serial.println("Objetivo(rad/s),Real(rad/s),PWM");
}

void loop() {
     double w_derecha, w_izquierda;
  static unsigned long lastMillis = 0;
 // --- GESTIÓN DE TIMEOUT (Rueda parada) ---
  unsigned long ahora = micros();
    // 1. Obtener el deltaTime de la ISR (usando sección crítica para evitar que cambie a mitad de lectura)
    noInterrupts();
    long dtL = deltaTimeLeft;
    long dtR = deltaTimeRight;
    interrupts();
    // 2. Calcular la velocidad instantánea
    double instantW_L = 0;
    double instantW_R =0;
  // 1. Capturamos el tiempo actual
  unsigned long ahora2 = micros();

// 2. Comprobamos si ha pasado mucho tiempo desde el último pulso (ej. 200ms = 200,000 us)
// Si pasa más de ese tiempo sin interrupciones, la velocidad es 0.
  if (ahora2 - timeAfterLeft > 200000) {
    instantW_L = 0;
  } 
  else {
    // Solo calculamos si hay un tiempo válido
    if (deltaTimeLeft > 0) 
      instantW_L = (2.0 * M_PI * 1000000.0) / (deltaTimeLeft * 20.0);
  }

  if (ahora2 - timeAfterRight > 200000) {
    instantW_R = 0;
  } 
  else {
    if (deltaTimeRight > 0) 
      instantW_R = (2.0 * M_PI * 1000000.0) / (deltaTimeRight * 20.0);
  }

// 3. Aplicar signo según la dirección (backI, backD)
  if (backI) instantW_L *= -1.0;
  if (backD) instantW_R *= -1.0;
  // IMPORTANTE: Limitar la velocidad máxima para evitar picos por ruido
    if (instantW_R > MAX_OPTIMAL_VEL) instantW_R = MAX_OPTIMAL_VEL;
    if (instantW_L > MAX_OPTIMAL_VEL) instantW_L = MAX_OPTIMAL_VEL;

    // 3. PASAR POR EL FILTRO
    meanFilterLeft.AddValue(instantW_L);
    meanFilterRight.AddValue(instantW_R);

    // 4. USAR EL VALOR FILTRADO PARA EL CONTROL Y TELEMETRÍA
    double wLeft = meanFilterLeft.GetFiltered();
    double wRight = meanFilterRight.GetFiltered();

    // 5. CINEMÁTICA DEL ROBOT (Estimación de velocidad del chasis)
    // v = w * r
    double vL = wLeft * miRobot.getRobotWheelRadius();
    double vR = wRight* miRobot.getRobotWheelRadius();
  
    double V_robot = (vR + vL) / 2.0;                    // Velocidad lineal (cm/s)
    double W_robot = (vR - vL) / miRobot.getRobotDiameter(); // Velocidad angular (rad/s)
    
    // DEBUG
  /*  static unsigned long ultimaImpresion = 0;
    if (millis() - ultimaImpresion > 250) { // Imprime cada 250ms
      DEBUG_PRINT("wLeft: "); 
      DEBUG_PRINT(wLeft);
      DEBUG_PRINT(" wRight: "); 
      DEBUG_PRINTLN(wRight);
      DEBUG_PRINT("v(cm/s): "); 
      DEBUG_PRINT(V_robot);
      DEBUG_PRINT(" w(rad/s): "); 
      DEBUG_PRINTLN(W_robot);
      ultimaImpresion = millis();
  }*/
  
#ifdef AJUSTEFF    
  if(pwm_output>=255)
    pwm_output=255;    
  // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if (millis() - lastMillis >= 1000) {
    lastMillis = millis();
    pwm_output+=10;
  
    if (TEST_RUEDA_DERECHA) {
  
            miRobot.moveRightWheel(pwm_output, VELOCIDAD_OBJETIVO, BACKWARDS);
            Serial.print("encoder: ");
            Serial.print(encoder_countRight);
            Serial.print("pwm: ");
            Serial.print(pwm_output);            
            Serial.print(", w_real: ");
            Serial.println(wRight);
  }
  else {
            miRobot.moveLeftWheel(pwm_output, VELOCIDAD_OBJETIVO, BACKWARDS);
            Serial.print("pwm: ");
            Serial.print(pwm_output);            
            Serial.print(", w_real: ");
            Serial.println(wLeft);
  }
  }

#endif

#ifdef TESTFF


  if(w_objetivo>=20){
    pwm_output=0;
 }
  // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if (millis() - lastMillis >= 1000) {
    lastMillis = millis();
     w_objetivo+=0.1;
     double w_real;

    if (TEST_RUEDA_DERECHA) {
            PID_RuedaR.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaR.feedForward(); 
            miRobot.moveRightWheel(pwm_output, wRight, BACKWARDS);
            Serial.print("pwm: ");
            Serial.print(pwm_output);
            Serial.print(", w_objetivo: ");
            Serial.println(w_objetivo);
            Serial.print(", w_real: ");
            Serial.println(wRight);
  }
  else  {
            PID_RuedaL.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaL.feedForward(); 
            miRobot.moveLeftWheel(pwm_output, wLeft, BACKWARDS);
            Serial.print("pwm: ");
            Serial.print(pwm_output);
            Serial.print(", w_objetivo: ");
            Serial.print(w_objetivo);
            Serial.print(", w_real: ");
            Serial.println(wLeft);
  }

  }

#endif


#ifdef AJUSTEPID


  if(w_objetivo>=20){
    pwm_output=0;
 }
  // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if (millis() - lastMillis >= 0.01) {
    lastMillis = millis();
     w_objetivo=10.0;
     int pwm_pid=0;

    if (TEST_RUEDA_DERECHA) {
            PID_RuedaR.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaR.feedForward();
            pwm_output+=PID_RuedaR.pid(wRight);
            pwm_output=constrain(pwm_output,MINPWM,MAXPWM);
            miRobot.moveRightWheel(pwm_output, wRight, BACKWARDS);
            Serial.print(", w_objetivo: ");
            Serial.println(w_objetivo);
            Serial.print(", w_real: ");
            Serial.println(wRight);
  }
  else  {
            PID_RuedaL.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaL.feedForward(); 
            //Serial.print("pwm_FF: ");
            //Serial.print(pwm_output);
            pwm_pid=PID_RuedaL.pid(wLeft);
            pwm_output+=pwm_pid;
            pwm_output=constrain(pwm_output,MINPWM,MAXPWM);
            miRobot.moveLeftWheel(pwm_output, wLeft, BACKWARDS);
            /*Serial.print("pwm: ");
            Serial.print(pwm_output);*/
            Serial.print("w_objetivo: ");
            Serial.print(w_objetivo);
            Serial.print(", w_real: ");
            Serial.println(wLeft);
  }

  }

#endif
}

/**
 * ISR para la rueda derecha.
 * Se activa en cada flanco de subida del sensor del encoder.
 */
void isrRight() {
  // 1. GESTIÓN DE REBOTES (Debouncing)
 unsigned long ahora=micros(); // Captura el tiempo actual del pulso
 
  // Solo procesamos el pulso si ha pasado suficiente tiempo (TIMEDEBOUNCE)
  // Esto filtra picos de voltaje o vibraciones mecánicas que darían velocidades falsas
  if(ahora-timeAfterRight > TIMEDEBOUNCE) {
    
    encoder_countRight++;      // Incrementa el contador total de pasos
    deltaTimeRight = ahora - timeAfterRight;
    timeAfterRight=ahora;
    }
}

/**
 * ISR para la rueda izquierda.
 * Realiza la misma lógica de filtrado y conteo para el motor izquierdo.
 */
void isrLeft() {
  // 1. GESTIÓN DE REBOTES (Debouncing)
  unsigned long ahora=micros(); // Captura el tiempo actual del pulso

  // Solo procesamos el pulso si ha pasado suficiente tiempo (TIMEDEBOUNCE)
  // Esto filtra picos de voltaje o vibraciones mecánicas que darían velocidades falsas
  if(ahora-timeAfterLeft > TIMEDEBOUNCE) {
    
    encoder_countLeft++;      // Incrementa el contador total de pasos
    deltaTimeLeft = ahora - timeAfterLeft;
    timeAfterLeft=ahora;
    }

}
