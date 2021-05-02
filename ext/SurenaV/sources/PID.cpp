#include "headers/PID.h"

PID::PID(double kp, double ki, double kd){
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
    
    this->intI = 0.0;
    this->prevoiusError = 0.0;

}

double PID::getOutput(double desiredValue, double currentValue){
    /*
        computes next plant input
    */
    double error = currentValue - desiredValue;
    this->intI += error;
    double deriv = error - this->prevoiusError;
    this->prevoiusError = error;

    return kp_ * error + ki_ * this->intI + kd_ * deriv;
}