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
#define ENCODER_CUADRATURA
#define ARDUINO_MKR
#define SMALL_BOT

#ifdef DEBUG_ENABLED
#define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)   
#define DEBUG_PRINTLN(...) 
#endif

// --- CONFIGURACIÓN DEL TEST ---
double VELOCIDAD_OBJETIVO = 0.0; // rad/s (ajusta según necesites)
bool TEST_RUEDA_DERECHA =false;   // true para derecha, false para izquierda
bool BACKWARDS= false; // true rueda hacia atras, false hacia adelante
// ------------------------------
// Uncomment only one
//#define  AJUSTEFF
//#define TESTFF
#define AJUSTEPID
//------------------------------
int pwm_output=0;
double w_objetivo=3.5;
robot miRobot;
controler PID_RuedaL, PID_RuedaR;


/// --- FILTROS Y ESTABILIZACIÓN ---
// Filtros de media móvil para suavizar las lecturas de velocidad de los encoders (ventana de 10 muestras)
MeanFilter<double> meanFilterRight(10);
MeanFilter<double> meanFilterLeft(10);

const double MAX_OPTIMAL_VEL=20; // Límite de seguridad en rad/s
static unsigned long lastMillis = 0, tpwmant=0,lastTime=0;
static unsigned long ultimaImpresion = 0, cambio_consigna=0;
void setup() {
  Serial.begin(9600);
  miRobot.pinSetup();
  miRobot.motorSetup();
  // Paso 1: Objetivo realista (aprox 2 vueltas por segundo)
  VELOCIDAD_OBJETIVO = 1.0; 
  PID_RuedaL.setSetPoint(VELOCIDAD_OBJETIVO);
  PID_RuedaR.setSetPoint(VELOCIDAD_OBJETIVO);

  PID_RuedaL.setFeedForwardParam(9.12,18.90);
  PID_RuedaR.setFeedForwardParam(8.5,19.0);
  // Paso 2: Solo proporcional (Kp). Ki y Kd a CERO.
  // Un Kp de 2.0 o 5.0 es un buen inicio para motores de bajo coste.
  PID_RuedaL.setControlerParam(5.0,2.5,0.0);
  PID_RuedaR.setControlerParam(4.0,2.5,0.0);
  // Configuración de interrupciones para encoders (necesario para calcular w real)
  #ifdef ENCODER_CUADRATURA
    pinMode(miRobot.getPinLeftEncoder(), INPUT_PULLUP); // Canal A Izq
    pinMode(miRobot.getPinLeftEncoderB() , INPUT_PULLUP); // Canal B Izq
    pinMode(miRobot.getPinRightEncoder() , INPUT_PULLUP); // Canal A Der
    pinMode(miRobot.getPinRightEncoderB() , INPUT_PULLUP); // Canal B Der
  
    // Interrumpimos en AMBOS canales para no perder ni un solo paso
    attachInterrupt(digitalPinToInterrupt(miRobot.getPinLeftEncoder()), isrL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(miRobot.getPinLeftEncoderB()), isrL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(miRobot.getPinRightEncoder()), isrR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(miRobot.getPinRightEncoderB()), isrR, CHANGE);

  #else
  
    attachInterrupt(digitalPinToInterrupt(miRobot.getPinLeftEncoder()), isrLeft, FALLING);
    attachInterrupt(digitalPinToInterrupt(miRobot.getPinRightEncoder()), isrRight, FALLING);
      // Inicializamos los tiempos para que la resta no dé valores extraños
    noInterrupts();
    timeAfterLeft = micros();
    timeAfterRight = micros();
    deltaTimeLeft = 0; 
    deltaTimeRight = 0;
    interrupts();
  

  #endif

    // Opcional: Si quieres que el filtro empiece "limpio"
    for(int i=0; i<10; i++) {
      meanFilterLeft.AddValue(0.0);
      meanFilterRight.AddValue(0.0);
  }

  Serial.println("Objetivo(rad/s),Real(rad/s),PWM");
}

void loop() {
     double w_derecha, w_izquierda;
     
   // 4. BUCLE DE LECTURA DE ENCODERS(Ejecutado cada SAMPLINGTIME, ej: 10ms)
   // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if (millis() - lastMillis >= 10) {
    #ifdef ENCODER_CUADRATURA
      noInterrupts();
      long encoderR=countsR;
      long encoderL=countsL;
      interrupts();
      // 2. Calcular la velocidad instantánea
      double instantW_L = 0;
      double instantW_R =0;
      // 1. Capturamos el tiempo actual
      unsigned long ahora = micros();
      unsigned long deltaTime=ahora-lastTime;
      instantW_L = (M_PI /1400.0) *(encoderL-encodercountLeftAnt)* 1000000 / (deltaTime);
      encodercountLeftAnt=encoderL;
      instantW_R=(M_PI /1400.0) *(encoderR-encodercountRightAnt)* 1000000 / (deltaTime);
      encodercountRightAnt=encoderR;
      lastTime=ahora;
        // IMPORTANTE: Limitar la velocidad máxima para evitar picos por ruido
    if (instantW_R > MAX_OPTIMAL_VEL) instantW_R = MAX_OPTIMAL_VEL;
    if (instantW_L > MAX_OPTIMAL_VEL) instantW_L = MAX_OPTIMAL_VEL;
        
    #else
   // 1. Obtener el deltaTime de la ISR (usando sección crítica para evitar que cambie a mitad de lectura)
    noInterrupts();
    long dtL = deltaTimeLeft;
    long dtR = deltaTimeRight;
    long encoderR=encoder_countRight;
    long encoderL=encoder_countLeft;
    interrupts();
    // 2. Calcular la velocidad instantánea
    double instantW_L = 0;
    double instantW_R =0;
  // 1. Capturamos el tiempo actual
  unsigned long ahora = micros();

// 2. Comprobamos si ha pasado mucho tiempo desde el último pulso (ej. 200ms = 200,000 us)
// Si pasa más de ese tiempo sin interrupciones, la velocidad es 0.
  if (ahora - timeAfterLeft > 200000000000) {
    instantW_L = 0;
  } 
  else {
    // Solo calculamos si hay un tiempo válido
    if (dtL > 0) {
       instantW_L = (M_PI /10.0) *(encoderL-encodercountLeftAnt)* 1000000 / (deltaTimeLeft );
       encodercountLeftAnt=encoderL;
    }
  }
  if (ahora - timeAfterRight > 2000000) {
    instantW_R = 0;
  } 
  else {
    if (dtR > 0) {
      instantW_R=(M_PI /10.0) *(encoderR-encodercountRightAnt)* 1000000 / (deltaTimeRight );
      encodercountRightAnt=encoderR;
    }
  }

// 3. Aplicar signo según la dirección (backI, backD)
  if (backI) instantW_L *= -1.0;
  if (backD) instantW_R *= -1.0;

 #endif
 
    // 3. PASAR POR EL FILTRO
    meanFilterLeft.AddValue(instantW_L);
    meanFilterRight.AddValue(instantW_R);

    lastMillis = millis();
  }
  // 4. USAR EL VALOR FILTRADO PARA EL CONTROL Y TELEMETRÍA
    double wLeft = meanFilterLeft.GetFiltered();
    double wRight = meanFilterRight.GetFiltered();

    // 5. CINEMÁTICA DEL ROBOT (Estimación de velocidad del chasis)
    // v = w * r
    double vL = wLeft * miRobot.getRobotWheelRadius();
    double vR = wRight* miRobot.getRobotWheelRadius();
  
    double V_robot = (vR + vL) / 2.0;                    // Velocidad lineal (cm/s)
    double W_robot = (vR - vL) / miRobot.getRobotDiameter(); // Velocidad angular (rad/s)
    
      
#ifdef AJUSTEFF    
    
  if(pwm_output>=255)
    pwm_output=254;    
    // Incremento el pwm cada segundo
    if ((millis()-tpwmant)> 1000 ){ 
      pwm_output+=5;
  
      if (TEST_RUEDA_DERECHA) {
    
              miRobot.moveRightWheel(pwm_output, VELOCIDAD_OBJETIVO, BACKWARDS);
               if ((millis() - ultimaImpresion )> 250) {
                Serial.print("encoder: ");
                #ifdef ENCODER_CUADRATURA
                  Serial.print(countsR);
                #else
                Serial.print(encoder_countRight);
                #endif
                Serial.print("pwm: ");
                Serial.print(pwm_output);            
                Serial.print(", w_real: ");
                Serial.println(wRight);}
              
    }
    else {
              miRobot.moveLeftWheel(pwm_output, VELOCIDAD_OBJETIVO, BACKWARDS);
                          Serial.print("encoder: ");
                #ifdef ENCODER_CUADRATURA
                  Serial.print(countsL);
                #else
                Serial.print(encoder_countLeft);
                #endif
              Serial.print("pwm: ");
              Serial.print(pwm_output);            
              Serial.print(", w_real: ");
              Serial.println(wLeft);
    }
    tpwmant=millis();
    }
  
#endif

#ifdef TESTFF


  if(w_objetivo>=40){
    pwm_output=0;
 }
  // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if (millis() - tpwmant>= 1000) {
    
     w_objetivo+=0.1;
     if(w_objetivo>13) w_objetivo=13;
     double w_real;

    if (TEST_RUEDA_DERECHA) {
            PID_RuedaR.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaR.feedForward(); 
            miRobot.moveRightWheel(pwm_output, wRight, BACKWARDS);
            /*Serial.print("pwm: ");
            Serial.print(pwm_output);*/
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
tpwmant=millis();
  }

#endif


#ifdef AJUSTEPID


  // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if((millis()-tpwmant)>10){
    
     int pwm_pid=0;

    if (TEST_RUEDA_DERECHA) {
            PID_RuedaR.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaR.feedForward();
            pwm_pid=PID_RuedaR.pid(wRight);
            pwm_output+=pwm_pid;
            pwm_output=constrain(pwm_output,MINPWM,MAXPWM);
            miRobot.moveRightWheel(pwm_output, wRight, BACKWARDS);
            if (millis() - ultimaImpresion > 50) {
              Serial.print("pwm: ");
              Serial.print(pwm_output);
              Serial.print("pwm_pid: ");
              Serial.print(pwm_pid);
              Serial.print(", w_objetivo: ");
              Serial.print(w_objetivo);
              Serial.print(", w_real: ");
              Serial.println(wRight);
              ultimaImpresion=millis();
                }
            if(millis()-cambio_consigna>10000){
              w_objetivo+=1;
              cambio_consigna=millis();
            }
            
  }
  else  {
            PID_RuedaL.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaL.feedForward(); 
            //Serial.print("pwm_FF: ");
            //Serial.print(pwm_output);
            wLeft=meanFilterLeft.GetFiltered();
            pwm_pid=PID_RuedaL.pid(wLeft);
            pwm_output+=pwm_pid;
            pwm_output=constrain(pwm_output,MINPWM,MAXPWM);
            miRobot.moveLeftWheel(pwm_output, wLeft, BACKWARDS);
            /*Serial.print("pwm: ");
            Serial.print(pwm_output);*/
            if (millis() - ultimaImpresion > 250) {
            
            Serial.print("pwm: ");
            Serial.print(pwm_output);
            Serial.print(" pwm_pid: ");
            Serial.print(pwm_pid);
            Serial.print("w_objetivo: ");
            Serial.print(w_objetivo);
            Serial.print(", w_real: ");
            Serial.println(wLeft);
             ultimaImpresion=millis();
             } 
            if(millis()-cambio_consigna>20000){
              w_objetivo+=1.0;
              cambio_consigna=millis();
            }
         }
   tpwmant=millis();

  }

#endif

}

#ifdef ENCODER_CUADRATURA


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

#else
/**
 * ISR para la rueda derecha.
 * Se activa en cada flanco de subida del sensor del encoder.
 */
void isrRight() {
  // 1. GESTIÓN DE REBOTES (Debouncing)
 unsigned long ahora=micros(); // Captura el tiempo actual del pulso
 
  // Solo procesamos el pulso si ha pasado suficiente tiempo (TIMEDEBOUNCE)
  // Esto filtra picos de voltaje o vibraciones mecánicas que darían velocidades falsas
  if(ahora-timeAfterRight >  DEBOUNCE_TIME) {
    
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
 if(ahora-timeAfterLeft >  DEBOUNCE_TIME) {
    
    encoder_countLeft++;      // Incrementa el contador total de pasos
    deltaTimeLeft = ahora - timeAfterLeft;
    timeAfterLeft=ahora;
    }

}

#endif
