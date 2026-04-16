
/*
 * ----------------------------------------------------------------------------
 * PROYECTO: Robotarium Hub (UCM)
 * ARCHIVO:  controller.h / controller.cpp
 * ----------------------------------------------------------------------------
 * DESCRIPCIÓN:
 * Implementación de los controladores de velocidad. Contiene la lógica del
 * algoritmo PID (Proporcional-Integral-Derivativo) y el modelo FeedForward
 * para la compensación de fricción y respuesta dinámica de los motores.
 * ----------------------------------------------------------------------------
 */
 
 #pragma once

#include <Arduino.h>

#define MINPWM 55
#define MAXPWM 255
#define VRMIN 4.85
#define VLMIN 4.64 
class controler
{
    public:
        controler();
        ~controler();
        void setControlerParam(double kp, double ki, double kd);
        int pid(double w);
        void setFeedForwardParam(double A, double B);
        int feedForward();
        void setSetPoint(double setPoint);
        double getSetPoint();
        void  setBack(bool back);
        bool getBack();
    private:
        unsigned long   currentTime, previousTime=0;;
        double          elapsedTime;
        double          error=0, lastError=0, cumError=0, rateError;
        double         setPoint;
        bool           back=false;
        double          kp, ki, kd;
        int            PWM;
        double feedForwardParam_A;
        double feedForwardParam_B;
        double maxIntegralError=14;
        

};
