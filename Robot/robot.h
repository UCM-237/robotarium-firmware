
/*
 * ----------------------------------------------------------------------------
 * ARCHIVO:  Robot.h
 * PROYECTO: Robotarium Hub (UCM)
 * ----------------------------------------------------------------------------
 * DESCRIPCIÓN:
 * Definición de la clase Robot. Actúa como el HAL (Hardware Abstraction Layer).
 * Contiene la declaración de métodos para control de motores y sensores.
 * ----------------------------------------------------------------------------
 */

 
#pragma once
#include <Arduino.h>
#include "config.h"

class robot
{
    public:
        void pinSetup();
        void motorSetup();

        void set_wheel_speed(int wheel, int direction, int speed) ;

        void moveForward(const int pinMotor[3], int speed);
        void moveBackward(const int pinMotor[3], int speed);
        void fullStop();
        void fullStopRightWheel();
        void fullStopLeftWheel();
        void moveRightWheel(int pwm, double w,bool back);
        void moveLeftWheel(int pwm, double w,bool back);
        double getRobotWheelDiameter();
        double getRobotWheelRadius();
        double getRobotDiameter();
        double getL();
        uint8_t getRobotID();

        int getPinLeftEncoder();
        int getPinRightEncoder();
        int getPinLeftEncoderB();
        int getPinRightEncoderB();

    private:
        int pinIN1;
        int pinIN2;
        int pinENA;
        int pinIN3;
        int pinIN4;
        int pinENB;
        int pinMotorRight[3];
        int pinMotorLeft[3];

        int pinLeftEncoder;
        int pinRightEncoder;
        int channelPinA_R;
        int channelPinB_R;
        int channelPinA_L;
        int channelPinB_L;

        
        double RobotWheelDiamter = ROBOT_WHEEL_DIAMETER ;
        double RobotWheelRadius = ROBOT_WHEEL_DIAMETER/2.0;
        double RobotDiameter = ROBOT_DIAMETER ;
        uint8_t robotID=ROBOT_ID;

        const int LEFT_WHEEL = 0;
        const int RIGHT_WHEEL = 1;





};
