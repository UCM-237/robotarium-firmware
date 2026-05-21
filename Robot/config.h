/*
 * ----------------------------------------------------------------------------
 * PROYECTO: Robotarium Hub (UCM)
 * ARCHIVO:  config.h
 * ----------------------------------------------------------------------------
 * DESCRIPCIÓN:
 * Definición de datos de cada robot.
 * Parámetros comunes para todos:
 * SAMPLINGTIME
 * DEBUG_ENABLED
 * 
 * Parámetros específicos de cada robot:
 * Tipo de Arduino ARDUINO_TYPE_MKR o ARDUINO_TYPE_NANO
 * Tipo de puente en H: H_BRIDGE_BLACK o H_BRIDGE_RED
 * Tipo de robot: BIGBOT o SMALLBOT
 * Tipo de encoder: ENCODER_CUADRATURA
 * 
 * Parámetros físicos del robot:
 * ROBOT_WHEEL_DIAMETER
 * ROBOT_DIAMETER 
 * 
 * Controladores de las ruedas:
 * Parámetros del FeedForward
 * A_L,B_L,A_R,B_R
 * Parámtros del controlador
 * Kp_R,Ki_R,Kd_R
 * Kp_L,Ki_L,Kd_L
 *  * ----------------------------------------------------------------------------
 */
 
 #pragma once
 
// 1. SELECCIÓN DEL ROBOT (Cambiar esto para cada robot)
/* 
#define ROBOT_ID 1 
#define ROBOT_ID 2 
#define ROBOT_ID 3 
#define ROBOT_ID 4 
#define ROBOT_ID 6 
#define ROBOT_ID 7 
#define ROBOT_ID 8 
#define ROBOT_ID 9 
#define ROBOT_ID 10 
*/

#define ROBOT_ID 6

// 2. PARÁMETROS GLOBALES (Comunes o por defecto)

#define SAMPLINGTIME 10
//#define DEBUG_ENABLED

// 3. CONFIGURACIONES ESPECÍFICAS POR ROBOT
// LEGOLAS
#if ROBOT_ID == 0 
    #define BIGBOT
    #define H_BRIDGE_RED
    #define ARDUINO_TYPE_NANO
    #define ROBOT_WHEEL_DIAMETER 6.7
    #define ROBOT_DIAMETER 14.5
    #define L 10
    // Calibración FF y PID específica para este robot

    #define MINPWM 55
    #define MAXPWM 255
    #define VRMIN 4.85
    #define VLMIN 4.64 
    
    // Rueda izquierda
    #define A_L 4.5
    #define B_L -9.2
    #define KP_L 5.0
    #define KI_L 2.5
    #define KD_L 0.0

    // Rueda derecha
     // Rueda izquierda
    #define A_R 4.5
    #define B_R -9.2
    #define KP_R 5.0
    #define KI_R 2.5
    #define KD_R 0.0
#elif ROBOT_ID == 1  //ARAGORN
    #define BIGBOT
    #define H_BRIDGE_RED
    #define ARDUINO_TYPE_NANO
    #define ROBOT_WHEEL_DIAMETER 6.7
    #define ROBOT_DIAMETER 14.5
    #define L 10
    // Calibración FF y PID específica para este robot
    
    #define MINPWM 55
    #define MAXPWM 255
    #define VRMIN 4.85
    #define VLMIN 4.64 
    
    // Rueda izquierda
    #define A_L 4.5
    #define B_L -9.2
    #define KP_L 5.0
    #define KI_L 2.5
    #define KD_L 0.0
    // Rueda derecha
     // Rueda izquierda
    #define A_R 4.5
    #define B_R -9.2
    #define KP_R 5.0
    #define KI_R 2.5
    #define KD_R 0.0
#elif ROBOT_ID == 3 //ARWEN
    #define BIGBOT
    #define H_BRIDGE_RED
    #define ROBOT_WHEEL_DIAMETER 6.7
    #define ROBOT_DIAMETER 14.5
    #define ARDUINO_TYPE_MKR
    #define L 10
    // Calibración FF y PID específica para este robot
    
    #define MINPWM 55
    #define MAXPWM 255
    #define VRMIN 4.85
    #define VLMIN 4.64 
    
    // Rueda izquierda
    #define A_L 4.5
    #define B_L -9.2
    #define KP_L 5.0
    #define KI_L 2.5
    #define KD_L 0.0
    // Rueda derecha
     // Rueda izquierda
    #define A_R 4.5
    #define B_R -9.2
    #define KP_R 5.0
    #define KI_R 2.5
    #define KD_R 0.0
#elif ROBOT_ID == 4 //BOROMIR
    #define BIGBOT
    #define H_BRIDGE_RED
    #define ROBOT_WHEEL_DIAMETER 6.7
    #define ROBOT_DIAMETER 14.5
    #define ARDUINO_TYPE_NANO
    #define L 10
    // Calibración FF y PID específica para este robot
    
    #define MINPWM 55
    #define MAXPWM 255
    #define VRMIN 4.85
    #define VLMIN 4.64 
    
    // Rueda izquierda
    #define A_L 4.5
    #define B_L -9.2
    #define KP_L 5.0
    #define KI_L 2.5
    #define KD_L 0.0
    // Rueda derecha
     // Rueda izquierda
    #define A_R 4.5
    #define B_R -9.2
    #define KP_R 5.0
    #define KI_R 2.5
    #define KD_R 0.0
#elif ROBOT_ID == 5 //GANDALF
    #define BIGBOT
    #define H_BRIDGE_RED
    #define ROBOT_WHEEL_DIAMETER 6.7
    #define ROBOT_DIAMETER 14.5
    #define ARDUINO_TYPE_NANO
    #define L 10.0
    // Calibración FF y PID específica para este robot
    
    #define MINPWM 30
    #define MAXPWM 255
    #define VRMIN 3.41
    #define VLMIN 12.36 
    
    // Rueda izquierda
    #define A_L 4.5
    #define B_L -9.2
    #define KP_L 5.0
    #define KI_L 2.5
    #define KD_L 0.0
    // Rueda derecha
     // Rueda izquierda
    #define A_R 4.5
    #define B_R -9.2
    #define KP_R 5.0
    #define KI_R 2.5
    #define KD_R 0.0
#elif ROBOT_ID == 6 //FRODO
    #define SMALLBOT
    #define H_BRIDGE_RED
    #define ENCODER_CUADRATURA
    #define ROBOT_WHEEL_DIAMETER 6.0
    #define ROBOT_DIAMETER 11.5
    #define ARDUINO_TYPE_MKR
    #define L 10
    #define TURN_CORRECTION_FACTOR  2.3
    #define MAX_ENCODER_STEPS 2800
    // Calibración FF y PID específica para este robot
    
  
    #define MINPWM 30
    #define MAXPWM 255
    #define VRMIN 3.41
    #define VLMIN 12.36 
    
    
    // Rueda izquierda
    #define A_L 9.12
    #define B_L 18.90
    #define KP_L 5.0
    #define KI_L 2.5
    #define KD_L 0.0
    // Rueda derecha
     // Rueda izquierda
    #define A_R 8.5
    #define B_R 19.0
    #define KP_R 4.0
    #define KI_R 2.5
    #define KD_R 0.0
#elif ROBOT_ID == 7 //GIMLI
    #define SMALLBOT
    #define H_BRIDGE_BLACK

    #define ROBOT_WHEEL_DIAMETER 6.7
    #define ROBOT_DIAMETER 14.5
    #define ARDUINO_TYPE_MKR
    #define L 10
    // Calibración FF y PID específica para este robot
      
    #define MINPWM 30
    #define MAXPWM 255
    #define VRMIN 3.41
    #define VLMIN 12.36 
    
    
    // Rueda izquierda
    #define A_L 16.98
    #define B_L -48.71
    #define KP_L 5.0
    #define KI_L 2.5
    #define KD_L 0.0
    // Rueda derecha
     // Rueda izquierda
    #define A_R 16.85
    #define B_R -53.48
    #define KP_R 4.0
    #define KI_R 2.5
    #define KD_R 0.0
    
#elif ROBOT_ID == 8 //MERRY
    #define SMALLBOT
    #define H_BRIDGE_BLACK

    #define ROBOT_WHEEL_DIAMETER 6.7
    #define ROBOT_DIAMETER 14.5
    #define ARDUINO_TYPE_MKR
    #define L 10
    // Calibración FF y PID específica para este robot
      
    #define MINPWM 30
    #define MAXPWM 255
    #define VRMIN 3.41
    #define VLMIN 12.36 
    
    // Rueda izquierda
    #define A_L 9.12
    #define B_L 18.90
    #define KP_L 5.0
    #define KI_L 2.5
    #define KD_L 0.0
    // Rueda derecha
     // Rueda izquierda
    #define A_R 8.5
    #define B_R 19.0
    #define KP_R 4.0
    #define KI_R 2.5
    #define KD_R 0.0
#elif ROBOT_ID == 9 //SAM
    #define SMALLBOT
    #define H_BRIDGE_RED
    #define ENCODER_CUADRATURA
    #define ROBOT_WHEEL_DIAMETER 6.7
    #define ROBOT_DIAMETER 14.5
    #define ARDUINO_TYPE_MKR
    #define L 10
    // Calibración FF y PID específica para este robot
      
    #define MINPWM 30
    #define MAXPWM 255
    #define VRMIN 3.41
    #define VLMIN 12.36 

    // Rueda izquierda
    #define A_L 9.12
    #define B_L 18.90
    #define KP_L 5.0
    #define KI_L 2.5
    #define KD_L 0.0
    // Rueda derecha
     // Rueda izquierda
    #define A_R 8.5
    #define B_R 19.0
    #define KP_R 4.0
    #define KI_R 2.5
    #define KD_R 0.0
// ... añadir casos hasta el ROBOT_ID 10
#endif
